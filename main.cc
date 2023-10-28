#include <concepts>
#include <limits>
#include <bit>
#include <type_traits>
#include <utility>
#include <array>
#include <span>

#include <cstddef>
#include <cstdint>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
#  include <stm32f411xe.h>
#pragma GCC diagnostic pop
#include <core_cm4.h>

namespace sfr
{
    void compile_time_evaluation_failure(char const*);

    template <std::unsigned_integral T> constexpr auto max = std::numeric_limits<T>::max();
    template <std::unsigned_integral T> constexpr auto digits = std::numeric_limits<T>::digits;
    template <std::unsigned_integral T>
    constexpr bool is_valid_mask(T x) noexcept {
        // has single continued pop, and empty not allowed.
        return digits<T> == std::popcount(x) + std::countr_zero(x) + std::countl_zero(x);
    }

    template <class R>
    concept as_sfr = std::unsigned_integral<R> && std::is_volatile_v<R>;

    using word = uint32_t;

    struct mask_value_list {
    public:
        template <size_t NN, size_t N = (NN >> 1)>
        consteval mask_value_list(word const (&mva)[NN]) noexcept
            : msk{max<word>}
            , val{}
            , clr{msk}
        {
            static_assert(0 < NN, "empty not allowed");
            static_assert(NN == 2*N, "even arguments required");
            [&]<size_t... I>(std::index_sequence<I...>) noexcept {
                std::array<word const, N> ma{mva[2*I]...};
                std::array<word const, N> va{(mva[2*I+1] << std::countr_zero(ma[I]))...};
                this->msk = (ma[I] | ...);
                this->val = (va[I] | ...);
                this->clr = ((ma[I] ^ va[I]) | ...);
                if (!(is_valid_mask(ma[I]) && ...)) {
                    compile_time_evaluation_failure("invalid mask");
                }
                if (std::popcount(msk) != (std::popcount(ma[I]) + ...)) {
                    compile_time_evaluation_failure("mask overlapped");
                }
                if (!(((va[I] & ~ma[I]) == 0) && ...)) {
                    compile_time_evaluation_failure("value overflowed");
                }
            }(std::make_index_sequence<N>());
        }

    public:
        template <std::integral... Args>
        consteval mask_value_list(Args... args) noexcept
            : mask_value_list{{static_cast<word>(args)...}}
        {
        }

    public:
        word msk;
        word val;
        word clr;
    };

    constexpr void rst(as_sfr auto& reg, mask_value_list mva) noexcept {
        reg = mva.val;
    }
    constexpr void set(as_sfr auto& reg, mask_value_list mva) noexcept {
        reg = (reg & ~mva.clr) | mva.val;
    }

    constexpr void dip(as_sfr auto const& reg, word msk = max<word>) noexcept {
        [[maybe_unused]] auto volatile tmp = reg & msk;
    }
    constexpr void spn(as_sfr auto const& reg, mask_value_list mva) noexcept {
        while (mva.val != (reg & mva.msk)) continue;
    }
    // WIP. to check with "mask_list" in the constant evaluation
    template <size_t N> [[nodiscard]]
    constexpr auto val(as_sfr auto const& reg, word const (&ma)[N]) noexcept {
        if constexpr (N == 1) {
            return (reg & ma[0]) >> std::countr_zero(ma[0]);
        }
        else {
            return [&]<size_t... I>(std::index_sequence<I...>) noexcept {
                return std::array<word, N> {
                    ((reg & ma[I]) >> std::countr_zero(ma[I]))...
                };
            }(std::make_index_sequence<N>());
        }
    }

} // ::sfr

extern "C"
{
    constexpr size_t SRAM_START = 0x2000'0000u;
    constexpr size_t SRAM_SIZE = 128u * 1024u;
    constexpr size_t SRAM_END = SRAM_START + SRAM_SIZE;
    constexpr size_t STACK_POINTER_INIT_ADDRESS = SRAM_END;

    int main(void);
    void reset_handler(void);
    void default_handler(void);
    void nmi_handler(void) __attribute__((weak, alias("default_handler")));
    void hard_fault_handler(void) __attribute__((weak, alias("default_handler")));
    void bus_fault_handler(void) __attribute__((weak, alias("default_handler")));
    void usage_fault_handler(void) __attribute__((weak, alias("default_handler")));
    void svcall_handler(void) __attribute__((weak, alias("default_handler")));
    void debug_monitor_handler(void) __attribute__((weak, alias("default_handler")));
    void pendsv_handler(void) __attribute__((weak, alias("default_handler")));
    void systick_handler(void) __attribute__((weak, alias("default_handler")));
    // ...continue adding device interrupt handlers

    uint32_t isr_vector[] __attribute__((section(".isr_vector"))) = {
        STACK_POINTER_INIT_ADDRESS,
        (uint32_t)&reset_handler,
        (uint32_t)&nmi_handler,
        (uint32_t)&hard_fault_handler,
        (uint32_t)&bus_fault_handler,
        (uint32_t)&usage_fault_handler,
        0,
        0,
        0,
        0,
        0,
        (uint32_t)&svcall_handler,
        (uint32_t)&debug_monitor_handler,
        0,
        (uint32_t)&pendsv_handler,
        (uint32_t)&systick_handler,
        // ...continue adding device interrupt handlers
    };

    void default_handler(void) {
        for(;;) continue;
    }

    extern constinit uint32_t const _etext;
    extern constinit uint32_t _sdata, _edata, _sbss, _ebss;

    void reset_handler() {
        size_t data_size = (&_edata - &_sdata) >> 2;
        auto flash_data = std::span<uint32_t const>{&_etext, data_size};
        auto sram_data = std::span<uint32_t>{&_sdata, data_size};
        size_t bss_size = (&_ebss - &_sbss) >> 2;
        auto bss = std::span<uint32_t>{&_sbss, bss_size};
        std::copy(flash_data.begin(), flash_data.end(), sram_data.begin());
        std::fill(bss.begin(), bss.end(), 0);
        sfr::set(SCB->CPACR, {
                3<<20, 3,
                3<<22, 3,
            });
        main();
    }
} // extern "C"

inline void sleep_for_ms(uint32_t delay_ms) noexcept {
    //uint32_t delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(delay).count();
    if (delay_ms != std::numeric_limits<decltype (delay_ms)>::max()) ++delay_ms;
    sfr::dip(SysTick->CTRL);
    while (delay_ms--) {
        sfr::spn(SysTick->CTRL, {SysTick_CTRL_COUNTFLAG_Msk, 1});
    }
}

#if 0
int main() {
    sfr::set(RCC->AHB1ENR, {RCC_AHB1ENR_GPIOAEN, 1});
    // do two dummy reads after enabling the peripheral clock, as per the errata
    sfr::dip(RCC->AHB1ENR);
    sfr::dip(RCC->AHB1ENR);
    sfr::set(GPIOA->MODER, {GPIO_MODER_MODER5, 1});
    for (;;) {
        GPIOA->ODR ^= GPIO_ODR_OD5;
        for (uint32_t i = 0; i < 1'000'000; ++i) continue;
    }
}
#else
int main() {
    sfr::set(RCC->APB2ENR, {RCC_APB2ENR_SYSCFGEN, 1});
    sfr::dip(RCC->APB2ENR);
    sfr::set(RCC->APB1ENR, {RCC_APB1ENR_PWREN, 1});
    sfr::dip(RCC->APB1ENR);
    sfr::set(SCB->AIRCR, {
        SCB_AIRCR_VECTKEY_Msk, 0x5fa,
        SCB_AIRCR_PRIGROUP_Msk, 7,
    });
    sfr::set(FLASH->ACR, {
            FLASH_ACR_ICEN, 1,
            FLASH_ACR_DCEN, 1,
            FLASH_ACR_PRFTEN, 1,
        });
    sfr::set(FLASH->ACR, {FLASH_ACR_LATENCY, 2});
    sfr::spn(FLASH->ACR, {FLASH_ACR_LATENCY, 2});
    sfr::set(PWR->CR, {PWR_CR_VOS, 3});
    sfr::set(RCC->CR, {
        RCC_CR_HSITRIM, 16,
        RCC_CR_HSION, 1,
    });
    sfr::spn(RCC->CR, {RCC_CR_HSIRDY, 1});
    sfr::set(RCC->PLLCFGR, {
        RCC_PLLCFGR_PLLSRC, 0,
        RCC_PLLCFGR_PLLM, 16,
        RCC_PLLCFGR_PLLN, 336,
        RCC_PLLCFGR_PLLP, 1,
    });
    sfr::set(RCC->CR, {RCC_CR_PLLON, 1});
    sfr::spn(RCC->CR, {RCC_CR_PLLON, 1});
    sfr::spn(PWR->CSR, {PWR_CSR_VOSRDY, 1});
    sfr::set(RCC->CFGR, {
        RCC_CFGR_HPRE, 0,
        RCC_CFGR_PPRE1, 4,
        RCC_CFGR_PPRE2, 0,
        RCC_CFGR_SW, 2,
    });
    sfr::spn(RCC->CFGR, {RCC_CFGR_SWS, 2});
    sfr::rst(SysTick->LOAD, {SysTick_LOAD_RELOAD_Msk, 84'000 - 1});
    sfr::rst(SysTick->VAL, {SysTick_VAL_CURRENT_Msk, 0});
    sfr::rst(SysTick->CTRL, {
        SysTick_CTRL_CLKSOURCE_Msk, 1,
        SysTick_CTRL_ENABLE_Msk, 1,
    });
    sfr::set(RCC->DCKCFGR, {RCC_DCKCFGR_TIMPRE, 0});
    sfr::set(RCC->AHB1ENR, {RCC_AHB1ENR_GPIOCEN, 1});
    sfr::set(RCC->AHB1ENR, {RCC_AHB1ENR_GPIOHEN, 1});
    sfr::set(RCC->AHB1ENR, {RCC_AHB1ENR_GPIOAEN, 1});
    sfr::set(RCC->AHB1ENR, {RCC_AHB1ENR_GPIOBEN, 1});
    sfr::dip(RCC->AHB1ENR);
    sfr::set(GPIOA->BSRR, {GPIO_BSRR_BR5, 1});
    sfr::set(SYSCFG->EXTICR[3], {SYSCFG_EXTICR4_EXTI13, 2});
    sfr::set(EXTI->EMR, {EXTI_EMR_MR13, 0});
    sfr::set(EXTI->IMR, {EXTI_IMR_MR13, 1});
    sfr::set(EXTI->RTSR, {EXTI_RTSR_TR13, 0});
    sfr::set(EXTI->FTSR, {EXTI_FTSR_TR13, 1});
    sfr::set(GPIOC->PUPDR, {GPIO_PUPDR_PUPD13, 0});
    sfr::set(GPIOC->MODER, {GPIO_MODER_MODER13, 0});
    sfr::set(GPIOA->OSPEEDR, {GPIO_OSPEEDR_OSPEED5, 0});
    sfr::set(GPIOA->OTYPER, {GPIO_OTYPER_OT5, 0});
    sfr::set(GPIOA->PUPDR, {GPIO_PUPDR_PUPD5, 0});
    sfr::set(GPIOA->MODER, {GPIO_MODER_MODER5, 1});
    sfr::set(RCC->APB1ENR, {RCC_APB1ENR_USART2EN, 1});
    sfr::dip(RCC->APB1ENR);
    sfr::set(GPIOA->OSPEEDR, {
        GPIO_OSPEEDR_OSPEED2, 3,
        GPIO_OSPEEDR_OSPEED3, 3,
    });
    sfr::set(GPIOA->OTYPER, {
        GPIO_OTYPER_OT2, 0,
        GPIO_OTYPER_OT3, 0,
    });
    sfr::set(GPIOA->PUPDR, {
        GPIO_PUPDR_PUPD2, 0,
        GPIO_PUPDR_PUPD3, 0,
    });
    sfr::set(GPIOA->AFR[0], {
        GPIO_AFRL_AFSEL2, 7,
        GPIO_AFRL_AFSEL3, 7,
    });
    sfr::set(GPIOA->MODER, {
        GPIO_MODER_MODER2, 2,
        GPIO_MODER_MODER3, 2,
    });
    if (!sfr::val(USART2->CR1, {USART_CR1_UE})) {
        sfr::set(USART2->CR1, {
            USART_CR1_M, 0,
            USART_CR1_PCE, 0,
            USART_CR1_PS, 0,
            USART_CR1_TE, 1,
            USART_CR1_RE, 1,
            USART_CR1_OVER8, 0,
        });
        sfr::set(USART2->CR2, {USART_CR2_STOP, 0});
        sfr::set(USART2->CR3, {
            USART_CR3_RTSE, 0,
            USART_CR3_CTSE, 0,
        });
        uint32_t sclk_freq;
        switch (sfr::val(RCC->CFGR, {RCC_CFGR_SWS})) {
            case 0:
                sclk_freq = HSI_VALUE;
                break;
            case 1:
                sclk_freq = HSE_VALUE;
                break;
            case 2: {
                auto input_freq = RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC ? HSE_VALUE : HSI_VALUE;
                auto [pllm, plln, pllp] = sfr::val(RCC->PLLCFGR, {
                    RCC_PLLCFGR_PLLM,
                    RCC_PLLCFGR_PLLN,
                    RCC_PLLCFGR_PLLP,
                });
                sclk_freq = (input_freq / pllm * plln) / ((pllp + 1) * 2);
                break;
            }
            default:
                sclk_freq = HSI_VALUE;
        }
        auto [hpre, ppre] = sfr::val(RCC->CFGR, {
            RCC_CFGR_HPRE,
            RCC_CFGR_PPRE1,
        });
        static constexpr uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
        static constexpr uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};
        uint32_t hclk_freq = sclk_freq >> AHBPrescTable[hpre];
        uint32_t pclk_freq = hclk_freq >> APBPrescTable[ppre];
        auto div = ((uint32_t)((((uint32_t)(pclk_freq))*25)/(4*((uint32_t)(115200)))));
        auto divmant = div/100;
        auto divfreq = (((div - divmant * 100) * 16) + 50) / 100;
        USART2->BRR = (divmant << 4) + (divfreq & 0xf0) + (divfreq & 0x0f);
    }
    sfr::set(USART2->CR2, {
        USART_CR2_LINEN, 0,
        USART_CR2_CLKEN, 0,
    });
    sfr::set(USART2->CR1, {USART_CR1_UE, 1});
    for (;;) {
        if (sfr::val(GPIOC->IDR, {GPIO_IDR_ID13})) {
            sleep_for_ms(500);
            sfr::rst(USART2->DR, {USART_DR_DR, '*'});
        }
        else {
            sleep_for_ms(125);
            sfr::rst(USART2->DR, {USART_DR_DR, '-'});
        }
        sfr::spn(USART2->SR, {USART_SR_TXE, 1});
        if (sfr::val(GPIOA->ODR, {GPIO_ODR_OD5})) {
            sfr::rst(GPIOA->BSRR, {
                GPIO_BSRR_BS5, 0,
                GPIO_BSRR_BR5, 1,
            });
        }
        else {
            sfr::rst(GPIOA->BSRR, {
                GPIO_BSRR_BS5, 1,
                GPIO_BSRR_BR5, 0,
            });
        }
    }
}
#endif
