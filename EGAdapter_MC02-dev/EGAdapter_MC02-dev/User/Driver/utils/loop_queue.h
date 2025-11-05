#pragma once
#include <cstdint>
#include <vector>

/**
 * @brief 简单环形队列(loop queue)模板
 *
 * @tparam T 队列元素类型
 *
 * @note
 *  - 本实现暂不加锁，因此在多线程/中断环境下需用户自行保证互斥。
 *  - 队列满时 push 会返回 false，pop 空队列会返回默认值。
 *  - 所有接口都带边界检查，避免越界访问。
 */
template <typename T>
class loop_queue {
 public:
	/**
	 * @brief 构造函数
	 * @param capacity 队列最大容量，默认 8
	 */
	explicit loop_queue(std::size_t capacity = 8)
			: buffer_(capacity), head_(0), tail_(0), size_(0), capacity_(capacity) {}

	/**
	 * @brief 获取队列当前元素数量
	 */
	std::size_t size() const { return size_; }

	/**
	 * @brief 获取队列容量
	 */
	std::size_t capacity() const { return capacity_; }

	/**
	 * @brief 判断队列是否为空
	 */
	bool empty() const { return size_ == 0; }

	/**
	 * @brief 判断队列是否已满
	 */
	bool full() const { return size_ == capacity_; }

	/**
	 * @brief 入队
	 * @param value 元素
	 * @return true 成功, false 队列已满
	 */
	bool push(const T &value) {
		if (full()) {
			//ERROR("loop_queue::push failed: queue full");
			return false;
		}
		buffer_[tail_] = value;
		tail_ = (tail_ + 1) % capacity_;
		++size_;
		return true;
	}

	/**
	 * @brief 出队并返回元素
	 * @return 若队列为空，返回默认构造的元素
	 */
	T pop() {
		if (empty()) {
			//ERROR("loop_queue::pop failed: queue empty");
			return T{};
		}
		T value = buffer_[head_];
		head_ = (head_ + 1) % capacity_;
		--size_;
		return value;
	}

	/**
	 * @brief 获取队头元素（不出队）
	 * @return 队头元素，若为空则返回默认值
	 */
	T front() const {
		if (empty()) {
			//ERROR("loop_queue::front failed: queue empty");
			return T{};
		}
		return buffer_[head_];
	}

	/**
	 * @brief 清空队列
	 */
	void clear() {
		head_ = 0;
		tail_ = 0;
		size_ = 0;
	}

 private:
	std::vector<T> buffer_;  ///< 底层存储
	std::size_t head_;       ///< 队头索引
	std::size_t tail_;       ///< 队尾索引
	std::size_t size_;       ///< 当前元素个数
	std::size_t capacity_;   ///< 队列容量
};
