#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
/// if memory_pool is too small, allocation may fail and return NULL
/// heap_blocks is like max variable(each variable is a block) in the pool
bool ta_init(const void *base, const void *limit, const size_t heap_blocks, const size_t split_thresh, const size_t alignment);
/// Like standard `malloc`, returns aligned pointer to address in heap space, or `NULL` if allocation failed.
void *ta_alloc(size_t num);
/// Like standard `calloc`, returns aligned pointer to zeroed memory in heap space, or `NULL` if allocation failed.
/// Allocate NMEMB elements of SIZE bytes each, all initialized to 0.
void *ta_calloc(size_t num, size_t size);
bool ta_free(void *ptr);
void *ta_realloc(void *ptr,size_t num);

size_t ta_num_free();
size_t ta_num_used();
size_t ta_num_fresh();
bool ta_check();

#ifdef __cplusplus
}
#endif
