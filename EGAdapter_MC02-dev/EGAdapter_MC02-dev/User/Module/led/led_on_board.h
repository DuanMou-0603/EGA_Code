//
// Author: Breezeee
// Date: 25-5-28
//

#ifndef LED_HPP
#define LED_HPP

// =============================== 调用库 ===============================
#include <optional>
#include "driver_spi.h"
// =============================== 变量区 ===============================

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} RGB;

// =============================== 函数声明 ===============================

// =============================== 类声明 ===============================

class LEDOnBoard {
 public:
	static LEDOnBoard &getInstance();

	static void loop();
	static void init();
	static void init(SPIInstance::Config &cfg);
	static void setColorRGB(uint8_t r, uint8_t g, uint8_t b);
	static void setColorHSV(float h, float s, float v);

	struct RGB {
		uint8_t r;
		uint8_t g;
		uint8_t b;
	};

	struct HSV {
		float h;
		float s;
		float v;
	};

	LEDOnBoard(const LEDOnBoard &) = delete;
	LEDOnBoard &operator=(const LEDOnBoard &) = delete;

 private:
	LEDOnBoard() = default;
	~LEDOnBoard() = default;

	static constexpr uint8_t WS2812_LowLevel = 0xC0; // 0码
	static constexpr uint8_t WS2812_HighLevel = 0xF0; // 1码

	void ctrl();
	void HSVtoRGB();

	bool inited_{false};

	std::optional<SPIInstance> spi_;

	RGB rgb_{0};
	HSV hsv_{0};
};
#endif // LED_HPP
