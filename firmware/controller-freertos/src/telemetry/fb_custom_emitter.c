#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "flatcc/flatcc_builder.h"

/** @author flatcc
 * Emits data to a conceptual deque by appending to either front or
 * back, starting from offset 0.
 *
 * Each emit call appends a strictly later or earlier sequence than the
 * last emit with same offset sign. Thus a buffer is gradually grown at
 * both ends. `len` is the combined length of all iov entries such that
 * `offset + len` yields the former offset for negative offsets and
 * `offset + len` yields the next offset for non-negative offsets.
 * The bulk of the data will be in the negative range, possibly all of
 * it. The first emitted emitted range will either start or end at
 * offset 0. If offset 0 is emitted, it indicates the start of clustered
 * vtables. The last positive (non-zero) offset may be zero padding to
 * place the buffer in a full multiple of `block_align`, if set.
 *
 * No iov entry is empty, 0 < iov_count <= FLATCC_IOV_COUNT_MAX.
 *
 * The source data are in general ephemeral and should be consumed
 * immediately, as opposed to caching iov.
 *
 * For high performance applications:
 *
 * The `create` calls may reference longer living data, but header
 * fields etc. will still be short lived. If an emitter wants to
 * reference data in another buffer rather than copying, it should
 * inspect the memory range. The length of an iov entry may also be used
 * since headers are never very long (anything starting at 16 bytes can
 * safely be assumed to be user provided, or static zero padding). It is
 * guaranteed that data pointers in `create` calls receive a unique slot
 * separate from temporary headers, in the iov table which may be used
 * for range checking or hashing (`create_table` is the only call that
 * mutates the data buffer). It is also guaranteed (with the exception
 * of `create_table` and `create_cached_vtable`) that data provided to
 * create calls are not referenced at all by the builder, and these data
 * may therefore de-facto be handles rather than direct pointers when
 * the emitter and data provider can agree on such a protocol. This does
 * NOT apply to any start/end/add/etc. calls which do copy to stack.
 * `flatcc_builder_padding_base` may be used to test if an iov entry is
 * zero padding which always begins at that address.
 *
 * Future: the emit interface could be extended with a type code
 * and return an existing object insted of the emitted if, for
 * example, they are identical. Outside this api level, generated
 * code could provide a table comparison function to help such
 * deduplication. It would be optional because two equal objects
 * are not necessarily identical. The emitter already receives
 * one object at time.
 *
 * Returns 0 on success and otherwise causes the flatcc_builder
 * to fail.
 */

#define STATIC_FB_BUFFER_SIZE (2 * 1024)                       // 4KB static payload buffer
#define FB_BUFFER_OFFSET_ZERO (STATIC_FB_BUFFER_SIZE * 3 / 4)  // Index of offset 0
static uint8_t fb_buffer[STATIC_FB_BUFFER_SIZE] = {0};

//                   ┌───────┬────────┐
//                   │   FB buffer    │ Emitter ctx
//                   └───────┼────────┘
//                     Data  │ Vtable (metadata)
//  - negative offset ◄───── │ ─────► + positive offset
//                           │
//                    center (at 0)
typedef struct {
    uint8_t* buf;
    size_t capacity;
    flatbuffers_soffset_t zero_offset;  // Center (offset +0) of the buffer
    flatbuffers_soffset_t min_offset;   // Tracks the lowest negative offset emitted
    flatbuffers_soffset_t max_offset;   // Tracks the highest positive offset emitted
} static_emitter_context_t;

/** @brief custom implementation of the emmiter function used by FB.
 *
 * In `flatcc`, we have two functions managing memory, the allocator and the emmiter. The allocator is used as a
 * "scratchpad", that `flatcc` uses to temporarily build up structures, before commiting them to the emitter. The
 * emitter takes the data produced by the allocator, and creates a contiguous sequence of binary data that is ready to
 * be sent.
 *
 * This function serves as the emitter function, and implementing a custom one allows us to predefine a static buffer in
 * which we can build the final FB. Be default, it would simply be allocating a very large sequence in ram and using
 * that instead. However, in our limited environment it is very hard to find a 2k contiguous free memory sequence.
 * Building our custom emmitter function ensures that that space is always available and ready to be used, as well as
 * prevents creating further memory fragmentation.
 *
 * @param emit_context Pointer to the data passed to the emitter.
 * @param iov Array of length `iov_count` containing the data+len that needs to be inserted into the emitter's buffer.
 * @param iov_count Length of the `iov` array.
 * @param offset Int32 representing the index where the element has to be inserted (can be positive or negative).
 * @param len The combined length of the `iov` entries.
 *
 */
int custom_builder_emit_fun(void* emit_context, const flatcc_iovec_t* iov, int iov_count, flatbuffers_soffset_t offset,
                            size_t len) {
    static_emitter_context_t* ctx = (static_emitter_context_t*)emit_context;

    // Update offsets
    if (offset < ctx->min_offset) ctx->min_offset = offset;
    if (offset + (flatbuffers_soffset_t)len > ctx->max_offset) {
        ctx->max_offset = offset + len;
    }

    // Bounds check
    if (ctx->zero_offset + ctx->min_offset < 0 ||
        ctx->zero_offset + ctx->max_offset > (flatbuffers_soffset_t)ctx->capacity) {
        return -1;  // Out of static buffer space
    }

    // Write iovec items to the buffer
    for (int i = 0; i < iov_count; ++i) {
        size_t elem_len = iov[i].iov_len;
        if (elem_len == 0) continue;

        uint8_t* dest = &ctx->buf[ctx->zero_offset + offset];

        // flatcc uses flatcc_builder_padding_base for zeroed alignment padding
        // FIXME: I don't understand why we are checking for "flatcc_builder_padding_base", since it is itself a static
        // buffer to a zero-padded block, so memcopying it should be safe.
        if (iov[i].iov_base == flatcc_builder_padding_base || iov[i].iov_base == NULL) {
            memset(dest, 0, elem_len);
        } else {
            memcpy(dest, iov[i].iov_base, elem_len);
        }

        // We move the offset over by what we just wrote to start writing the next entry.
        // safe because it is guaranteed that only half of the integer is used.
        offset += (flatbuffers_soffset_t)elem_len;
    }

    return 0;
}

int init_flatbuffers(flatcc_builder_t* builder) {
    static static_emitter_context_t emit_ctx = {
        .buf = fb_buffer,
        .capacity = STATIC_FB_BUFFER_SIZE,
        .zero_offset = FB_BUFFER_OFFSET_ZERO,
        .min_offset = 0,
        .max_offset = 0,
    };

    // Initialize builder with custom emitter
    int ret = flatcc_builder_custom_init(
        builder,
        custom_builder_emit_fun,  // Customized emmiter function that will build the final packet gradually. This is
                                  // required because else we would constantly have to have a contiguous block of memory
                                  // of about 2k, which is not realistic in our environment.
        &emit_ctx,                // Pointer to the data that is passed to the emitter
        NULL,                     // use default allocator
        NULL                      // use default allocator context
    );

    if (ret != 0) {
        return -1;  // failure
    }

    return 0;
}

void* get_final_buffer(flatcc_builder_t* builder, size_t* out_size) {
    static_emitter_context_t* ctx = (static_emitter_context_t*)builder->emit_context;
    *out_size = (size_t)(ctx->max_offset - ctx->min_offset);
    return (void*)(&ctx->buf[ctx->zero_offset + ctx->min_offset]);
}

void reset_emitter(flatcc_builder_t* builder) {
    static_emitter_context_t* ctx = (static_emitter_context_t*)builder->emit_context;
    if (!ctx) return;

    /* Reset offset trackers back to zero */
    ctx->min_offset = 0;
    ctx->max_offset = 0;

#ifdef DEBUG
    memset(ctx->buf, 0, ctx->capacity);
#endif
}