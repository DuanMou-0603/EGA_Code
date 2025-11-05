//
// Author: Breezeee
// Date: 25-5-28
//

// =============================== 引入头文件 ===============================
#include "led_on_board.h"

// // =============================== 变量区 ===============================
//
// // =============================== 函数实现 ===============================
LEDOnBoard &LEDOnBoard::getInstance() {
	static LEDOnBoard instance;
	return instance;
}

void LEDOnBoard::init() {
	if (getInstance().inited_) {
		return;
	}
	SPIInstance::Config cfg;
	cfg.handle = &hspi6;
	cfg.cs_port = nullptr;
	cfg.cs_pin = 0;
	cfg.effect_pin_state_ = GPIO_PIN_RESET;
	cfg.type = SPIInstance::BLOCK;
	cfg.mode = SPIInstance::TX_ONLY;
	cfg.is_master = true;
	cfg.tx_size = 24;
	cfg.rx_size = 0;
	cfg.rx_cbk = nullptr;
	cfg.tx_cbk = nullptr;

	getInstance().spi_.emplace(cfg);
	getInstance().inited_ = true;
}

void LEDOnBoard::init(SPIInstance::Config &cfg) {
	if (getInstance().inited_) {
		return;
	}
	getInstance().spi_.emplace(cfg);
	getInstance().inited_ = true;
}

void LEDOnBoard::loop() {
	static float hue = 0.0f;

	// hue 取值范围 0~360，用于控制颜色
	hue += 2.0f;
	if (hue >= 360.0f) hue -= 360.0f;

	setColorHSV(hue, 1.0f, 0.3f);
	getInstance().HSVtoRGB();
	getInstance().ctrl();
}

void LEDOnBoard::ctrl() {
	uint8_t res = 0;
	uint8_t txbuf[24]{0};
	for (int i = 0; i < 8; i++) {
		txbuf[7 - i] = (((rgb_.g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
		txbuf[15 - i] = (((rgb_.r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
		txbuf[23 - i] = (((rgb_.b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
	}
	spi_->trans(&res, 1);
	spi_->trans(txbuf, 24);
	for (int i = 0; i < 100; i++) {
		spi_->trans(&res, 1);
	}
}

void LEDOnBoard::HSVtoRGB() {
	float h = hsv_.h;
	float s = hsv_.s;
	float v = hsv_.v;
	float r{0}, g{0}, b{0};
	int i{0};
	float f{0}, p{0}, q{0}, t{0};

	if (s == 0.0f) {
		r = g = b = v;
	} else {
		h /= 60.0f;
		i = (int)h;
		f = h - i;
		p = v * (1.0f - s);
		q = v * (1.0f - s * f);
		t = v * (1.0f - s * (1.0f - f));

		switch (i % 6) {
			case 0:
				r = v;
				g = t;
				b = p;
				break;
			case 1:
				r = q;
				g = v;
				b = p;
				break;
			case 2:
				r = p;
				g = v;
				b = t;
				break;
			case 3:
				r = p;
				g = q;
				b = v;
				break;
			case 4:
				r = t;
				g = p;
				b = v;
				break;
			case 5:
				r = v;
				g = p;
				b = q;
				break;
		}
	}

	rgb_.r = (uint8_t)(r * 255);
	rgb_.g = (uint8_t)(g * 255);
	rgb_.b = (uint8_t)(b * 255);
}

void LEDOnBoard::setColorHSV(const float h, const float s, const float v) {
	getInstance().hsv_.h = h;
	getInstance().hsv_.s = s;
	getInstance().hsv_.v = v;
	getInstance().HSVtoRGB();
	getInstance().ctrl();
}
void LEDOnBoard::setColorRGB(uint8_t r, uint8_t g, uint8_t b) {
	getInstance().rgb_.r = r;
	getInstance().rgb_.g = g;
	getInstance().rgb_.b = b;
	getInstance().ctrl();
}
