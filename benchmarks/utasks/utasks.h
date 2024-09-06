#ifndef UTASKS_H
#define UTASKS_H

#include <stdint.h>
#include <vector>
#include <list>
#include <cstddef>
#include <cassert>
#include <stdio.h>
#include <atomic>
#include "util.h"


class runner_t;

class mutex_t { // can't use std::mutex if we don't have pthreads
public:
  mutex_t() : next_ticket(0), now_serving(0) { }
  void lock() {
    size_t my_ticket = next_ticket.fetch_add(1, std::memory_order_relaxed);
    while (my_ticket != now_serving.load(std::memory_order_acquire)) { }
  }
  void unlock() { now_serving.fetch_add(1, std::memory_order_release); }
private:
  std::atomic<size_t> next_ticket;
  std::atomic<size_t> now_serving;
};

mutex_t printf_lock;

#define PRINTF(...) ({							\
      printf_lock.lock();						\
      printf("hart %ld: ", read_csr(mhartid));                          \
      printf(__VA_ARGS__);						\
      printf_lock.unlock();						\
})

class allocator_t {
public:
  virtual void* allocate(size_t n) = 0;
  virtual void deallocate(void* buf) = 0;
};


mutex_t heap_allocator_lock;
class heap_allocator_t : public allocator_t {
public:
  void* allocate(size_t n) {
    heap_allocator_lock.lock();
    void* buf = malloc(n);
    heap_allocator_lock.unlock();
    return buf;
  }
  void deallocate(void* buf) {
    heap_allocator_lock.lock();
    free(buf);
    heap_allocator_lock.unlock();
  }
};

class region_allocator_t : public allocator_t {
public:
  region_allocator_t(void* base, size_t size) : base(base), size(size), tail(0) { }
  void* allocate(size_t n) {
    lock.lock();
    if (tail + n > size) {
      lock.unlock();
      PRINTF("illegal allocation size %ld\n", size);
      exit(1);
    }
    void* r = (uint8_t*)base + tail;
    tail += n;
    lock.unlock();
    return r;
  }
  void deallocate(void* buf) { }
private:
  mutex_t lock;
  void* base;
  size_t size;
  size_t tail;
};

class alignas(64) task_t {
  friend class runner_t;
public:
  task_t() : may_finish(false) { }
  void set_may_finish() { may_finish = true; }
  bool __attribute__ ((noinline)) is_finished() { return may_finish && !has_work(); }
  void wait_for_finished() { while (!this->is_finished()) { }; }
  virtual void propagate_finished() = 0;
  virtual void assign_runner(runner_t* runner) { };

private:
  virtual void run() = 0;
  virtual bool has_work() = 0;
protected:
  bool may_finish;
};

class circular_buffer_pointers_t {
public:
  circular_buffer_pointers_t() : head_wide(0), tail_wide(0), size(0) { }
  size_t head_wide;
  size_t tail_wide;
  size_t size;
};

template <typename T>
class alignas(64) circular_buffer_t {
public:
  circular_buffer_t(size_t capacity, void* buffer, circular_buffer_pointers_t* pointers) : buffer((T*)buffer), capacity(capacity), pointers(pointers) {
    if (capacity < 4 || (capacity & (capacity-1)) != 0) {
      PRINTF("Illegal capacity %ld\n", capacity);
      exit(1);
    }
    mask = capacity - 1;
  }
  ~circular_buffer_t() {
    delete[] buffer;
  }

  std::pair<T*, size_t> push(size_t n) {
    size_t tail_wide_read, size_read;
    size_t mask = this->mask;
    asm volatile("amoadd.d %[rd], %[incr], (%[addr])\n" : [rd]"=r"(tail_wide_read) : [incr]"r"(n), [addr]"r"(&pointers->tail_wide));
    asm volatile("amoadd.d %[rd], %[incr], (%[addr])\n" : [rd]"=r"(size_read)      : [incr]"r"(n), [addr]"r"(&pointers->size));
    tail_wide_read += n;
    size_read += n;
    size_t tail = tail_wide_read & mask;
    size_t head = pointers->head_wide & mask;
    size_t remaining_capacity = capacity - size_read;
    size_t limit = ((head > tail) ? head : capacity) - tail;
    size_t r = (remaining_capacity > limit) ? limit : remaining_capacity;
    return std::make_pair(buffer + tail, r);
  }

  std::pair<T*, size_t> pop(size_t n) {
    size_t head_wide_read, size_read;
    size_t mask = this->mask;
    asm volatile("amoadd.d %[rd], %[incr], (%[addr])\n" : [rd]"=r"(head_wide_read) : [incr]"r"(n)     , [addr]"r"(&pointers->head_wide));
    asm volatile("amoadd.d %[rd], %[incr], (%[addr])\n" : [rd]"=r"(size_read)      : [incr]"r"((~n)+1), [addr]"r"(&pointers->size));
    head_wide_read += n;
    size_read -= n;
    size_t head = head_wide_read & mask;
    size_t tail = pointers->tail_wide & mask;
    size_t limit = ((tail > head) ? tail : capacity) - head;
    size_t r = (size_read > limit) ? limit : size_read;
    return std::make_pair(buffer + head, r);
  }

  bool busy() { return pointers->size != 0; }

  T* buffer;
  size_t mask;
  size_t capacity;
  circular_buffer_pointers_t* pointers;
};

template <typename T, typename U>
class circular_buffer_helper_t {
public:
  static std::tuple<T*, U*, size_t> push_pop(
                                             circular_buffer_t<T>* const source,
                                             circular_buffer_pointers_t* source_pointers,
                                             circular_buffer_t<U>* const sink,
                                             circular_buffer_pointers_t* sink_pointers,
                                             size_t n) {
    size_t sink_tail_wide_read, sink_size_read;
    size_t source_head_wide_read, source_size_read;
    asm volatile("amoadd.d %[rd], %[incr], (%[addr])\n" : [rd]"=r"(sink_tail_wide_read)   : [incr]"r"(n)     , [addr]"r"(&sink_pointers->tail_wide));
    asm volatile("amoadd.d %[rd], %[incr], (%[addr])\n" : [rd]"=r"(source_head_wide_read) : [incr]"r"(n)     , [addr]"r"(&source_pointers->head_wide));
    asm volatile("amoadd.d %[rd], %[incr], (%[addr])\n" : [rd]"=r"(sink_size_read)        : [incr]"r"(n)     , [addr]"r"(&sink_pointers->size));
    asm volatile("amoadd.d %[rd], %[incr], (%[addr])\n" : [rd]"=r"(source_size_read)      : [incr]"r"((~n)+1), [addr]"r"(&source_pointers->size));
    size_t sink_head_wide = sink_pointers->head_wide;
    size_t source_tail_wide = source_pointers->tail_wide;

    asm volatile ("\n");
    size_t sink_mask = sink->mask;
    size_t source_mask = source->mask;
    size_t sink_capacity = sink->capacity;
    size_t source_capacity = source->capacity;

    asm volatile ("add %[rd], %[src], %[incr]\n" : [rd]"=r"(sink_tail_wide_read)   : [src]"r"(sink_tail_wide_read)  , [incr]"r"(n));
    asm volatile ("add %[rd], %[src], %[incr]\n" : [rd]"=r"(source_head_wide_read) : [src]"r"(source_head_wide_read), [incr]"r"(n));
    asm volatile ("add %[rd], %[src], %[incr]\n" : [rd]"=r"(sink_size_read)        : [src]"r"(sink_size_read)       , [incr]"r"(n));
    asm volatile ("sub %[rd], %[src], %[incr]\n" : [rd]"=r"(source_size_read)      : [src]"r"(source_size_read)     , [incr]"r"(n));

    T* source_buffer = source->buffer;
    U* sink_buffer = sink->buffer;
    size_t remaining_capacity = sink_capacity - sink_size_read;
    size_t sink_tail = sink_tail_wide_read & sink_mask;
    size_t sink_head = sink_head_wide & sink_mask;
    size_t sink_limit = ((sink_head > sink_tail) ? sink_head : sink_capacity) - sink_tail;
    size_t sink_r = (remaining_capacity > sink_limit) ? sink_limit : remaining_capacity;

    size_t source_head = source_head_wide_read & source_mask;
    size_t source_tail = source_tail_wide & source_mask;
    size_t source_limit = ((source_tail > source_head) ? source_tail : source_capacity) - source_head;
    size_t source_r = (source_size_read > source_limit) ? source_limit : source_size_read;

    size_t r = (sink_r > source_r) ? source_r : sink_r;
    return std::make_tuple(source_buffer + source_head, sink_buffer + sink_tail, r);
  }
};

template <typename T> class sink_t;
template <typename T> class source_t;

class runner_t {
public:
  runner_t(size_t id, allocator_t* allocator) : id(id), task_count(0), allocator(allocator) { };
  void run() {
    while (1) {
      task_t* scheduled_task = schedule_task();
      if (scheduled_task) {
	scheduled_task->run();
      }
    }
  }

  void add_task(task_t* task) {
    task->assign_runner(this);
    task_lock.lock();
    tasks.push_back(task);
    task_count++;
    task_lock.unlock();
  }

  bool idle() { return task_count.load() == 0; }

  allocator_t* get_allocator() { return allocator; }

private:
  task_t* schedule_task() {
    task_lock.lock();
    for (auto& t : tasks) {
      if (t->has_work()) {
	task_lock.unlock();
	return t;
      }
    }
    std::list<task_t*>::iterator it = tasks.begin();
    while (it != tasks.end()) {
      if ((*it)->is_finished()) {
        (*it)->propagate_finished();
	it = tasks.erase(it);
	task_count--;
      }
      it++;
    }
    task_lock.unlock();
    return nullptr;
  }

  size_t id;
  mutex_t task_lock;
  std::atomic<size_t> task_count;
  std::list<task_t*> tasks;
  allocator_t* allocator;
};


template <typename T>
class sink_t : virtual public task_t {
  friend class source_t<T>;
public:
  sink_t(size_t buffer_size) : buffer_size(buffer_size), buffer(nullptr) { }
  ~sink_t() {
    runner->get_allocator()->deallocate(buffer_data);
    runner->get_allocator()->deallocate(buffer);
  }
  void assign_runner(runner_t* runner) {
    this->runner = runner;
    void* pointers_buff = runner->get_allocator()->allocate(sizeof(circular_buffer_pointers_t));
    circular_buffer_pointers_t* pointers = new (pointers_buff) circular_buffer_pointers_t;
    buffer_data = runner->get_allocator()->allocate(buffer_size * sizeof(T));
    buffer = new circular_buffer_t<T>(buffer_size, buffer_data, pointers);
  }
  size_t buffer_size;
  alignas(64) circular_buffer_t<T>* buffer;
private:
  runner_t* runner;
  void* buffer_data;
};

template <typename T>
class source_t : virtual public task_t {
public:
  source_t() : next(nullptr), output(nullptr) { }
  void chain(sink_t<T>* next) {
    if (this->next) {
      PRINTF("Failed chain\n");
      exit(1);
    }
    this->next = next;
  }
  void terminate(T* out) {
    if (this->next) {
      PRINTF("Failed chain\n");
      exit(1);
    }
    output = out;
  }
  void propagate_finished() {
    if (this->next) { this->next->set_may_finish(); }
  }
protected:
  sink_t<T>* next;
  alignas(64) T* output;
};


template <typename T, typename U>
class pipe_task_t : public source_t<U>, public sink_t<T> {
public:
  pipe_task_t(size_t buffer_size, size_t max_chunk) : max_chunk(max_chunk), sink_t<T>(buffer_size) { }
  bool has_work() { return this->buffer->busy(); }
  virtual size_t kernel(T* in, U* out, size_t n) = 0;
private:
  void run() {
    bool has_next = this->next != nullptr;
    circular_buffer_t<U>* next_buffer = has_next ? this->next->buffer : nullptr;
    size_t max_chunk = this->max_chunk;
    circular_buffer_t<T>* buffer = this->buffer;
    if (has_next) {
      circular_buffer_pointers_t* source_pointers = buffer->pointers;
      circular_buffer_pointers_t* next_pointers = next_buffer->pointers;
      std::tuple<T*, T*, size_t> r = circular_buffer_helper_t<T, U>::push_pop(buffer, source_pointers,
                                                                              next_buffer, next_pointers,
                                                                              0);
      T* input = std::get<0>(r);
      T* output = std::get<1>(r);
      size_t n = std::get<2>(r);
      while (n > 0) {
        size_t completed = 0;
        while (completed < max_chunk && n > 0) {
          size_t finished = kernel(input, output, n);
          n -= finished;
          completed += finished;
          input += finished;
          output += finished;
        }
	asm volatile("fence");
	r = circular_buffer_helper_t<T, U>::push_pop(buffer, source_pointers,
                                                     next_buffer, next_pointers,
                                                     completed);
	input = std::get<0>(r);
	output = std::get<1>(r);
	n = std::get<2>(r);
	if (completed == 0) break;
      }
    } else {
      std::pair<T*, size_t> pop = buffer->pop(0);
      T* input = pop.first;
      size_t n = pop.second;
      while (n > 0) {
        size_t completed = 0;
        while (completed < max_chunk && n > 0) {
          size_t finished = kernel(input, this->output, n);
          n -= finished;
          completed += finished;
          input += finished;
          this->output += finished;
        }
	asm volatile("fence");
	pop = buffer->pop(completed);
        input = pop.first;
	n = pop.second;
	if (completed == 0) break;
      }
    }
  }
  size_t max_chunk;
};

template <typename T>
class source_task_t : public source_t<T> {
public:
  source_task_t(size_t max_chunk) : max_chunk(max_chunk) { }
  virtual size_t kernel(T* out, size_t n) = 0;
  virtual bool has_work() = 0;
private:
  void run() {
    bool has_next = this->next != nullptr;
    assert(has_next);
    circular_buffer_t<T>* next_buffer = has_next ? this->next->buffer : nullptr;
    size_t max_chunk = this->max_chunk;

    std::pair<T*, size_t> push = next_buffer->push(0);
    T* output = push.first;
    size_t n = push.second;
    while (n > 0) {
      size_t completed = 0;
      while (completed < max_chunk && n > 0) {
        size_t finished = kernel(output, n);
        n -= finished;
        completed += finished;
        output += finished;
        if (finished == 0) break;
      }
      asm volatile("fence");
      push = next_buffer->push(completed);
      output = push.first;
      n = push.second;
      if (completed == 0) break;
    }
  }
  size_t max_chunk;
};

template <typename T>
class sink_task_t : public sink_t<T> {
public:
  sink_task_t(size_t buffer_size, size_t max_chunk) : max_chunk(max_chunk), sink_t<T>(buffer_size) { }
  bool has_work() { return this->buffer->busy(); }
  virtual size_t kernel(T* in, size_t n) = 0;
private:
  void run() {
    size_t max_chunk = this->max_chunk;
    circular_buffer_t<T*> buffer = this->buffer;
    std::pair<T*, size_t> pop = buffer->pop(0);
    T* input = pop.first;
    size_t n = pop.second;
    while (n > 0) {
      size_t completed = 0;
      while (completed < max_chunk && n > 0) {
        size_t finished = kernel(input, n);
        n -= finished;
        completed += finished;
        input += finished;
      }
      asm volatile("fence");
      pop = buffer->pop(completed);
      input = pop.first;
      n = pop.second;
      if (completed == 0) break;
    }
  }
  size_t max_chunk;
};



#endif
