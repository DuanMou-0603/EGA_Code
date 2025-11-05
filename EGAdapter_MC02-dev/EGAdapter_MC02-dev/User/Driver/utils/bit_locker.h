#pragma once
// sal
//#include "log.h"

#include <cstdint>
#include <memory>
#include <atomic>

#define DEBUG_DEADLOCK(message) \
    do                          \
    {                           \
        while (1)               \
            LOGERROR(message);  \
    } while (0)


/**
 * 一个适配H7处理器的互斥锁
 */
class bit_locker_h7 {
 private:
	std::atomic<uint8_t> bit{0};

 public:
	bool try_lock() {
		uint8_t expected = 0;
		return bit.compare_exchange_strong(expected, 1,
																			 std::memory_order_acquire,
																			 std::memory_order_relaxed);
	}

	void unlock() {
		bit.store(0, std::memory_order_release);
	}
};

/* 互斥锁,仅单核处理器可用 */
class bit_locker
{
    friend class lock_guard;

private:
    volatile uint8_t bit = 0;

public:
    bit_locker() = default;
    ~bit_locker() = default;

    bool try_lock()
    {
        if (!bit)
            return (bit = 1);
        else
            return false;
    }
};

/* 配合互斥锁,完成锁资源操作后自动析构释放锁 */
class lock_guard
{
private:
    volatile uint8_t *bit_ptr = nullptr;

public:
    lock_guard(bit_locker &locker) : bit_ptr(&locker.bit){};
    ~lock_guard() { *bit_ptr = 0; };
};