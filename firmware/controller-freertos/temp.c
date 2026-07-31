#define __flatbuffers_build_struct(NS, N, S, A, FID, TFID)                                                    \
    __##NS##define_struct_primitives(NS, N) typedef NS##ref_t N##_ref_t;                                      \
    static inline N##_t* N##_start(NS##builder_t* B) { return (N##_t*)flatcc_builder_start_struct(B, S, A); } \
    static inline N##_ref_t N##_end(NS##builder_t* B) {                                                       \
        if (!NS##is_native_pe()) {                                                                            \
            N##_to_pe((N##_t*)flatcc_builder_struct_edit(B));                                                 \
        }                                                                                                     \
        return flatcc_builder_end_struct(B);                                                                  \
    }                                                                                                         \
    static inline N##_ref_t N##_end_pe(NS##builder_t* B) { return flatcc_builder_end_struct(B); }             \
    static inline N##_ref_t N##_create(NS##builder_t* B __##N##_formal_args) {                                \
        N##_t* _p = N##_start(B);                                                                             \
        if (!_p) return 0;                                                                                    \
        N##_assign_to_pe(_p __##N##_call_args);                                                               \
        return N##_end_pe(B);                                                                                 \
    }                                                                                                         \
    static inline N##_ref_t N##_clone(NS##builder_t* B, N##_struct_t p) {                                     \
        N##_t* _p;                                                                                            \
        __flatbuffers_memoize_begin(B, p);                                                                    \
        _p = N##_start(B);                                                                                    \
        if (!_p) return 0;                                                                                    \
        N##_copy(_p, p);                                                                                      \
        __flatbuffers_memoize_end(B, p, N##_end_pe(B));                                                       \
    }                                                                                                         \
    __flatbuffers_build_vector(NS, N, N##_t, S, A) __flatbuffers_build_struct_root(NS, N, A, FID, TFID)

///////////////////////////////////////////////////////

static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_to_pe(
    FripuckProtocol_Sensors_ImuData_t* p) {
    if (!(1)) {
        FripuckProtocol_Sensors_ImuData_copy_to_pe(p, p);
    };
    return p;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_from_pe(
    FripuckProtocol_Sensors_ImuData_t* p) {
    if (!(1)) {
        FripuckProtocol_Sensors_ImuData_copy_from_pe(p, p);
    };
    return p;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_clear(
    FripuckProtocol_Sensors_ImuData_t* p) {
    return (FripuckProtocol_Sensors_ImuData_t*)memset(p, 0, FripuckProtocol_Sensors_ImuData__size());
}
typedef flatbuffers_ref_t FripuckProtocol_Sensors_ImuData_ref_t;
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_start(flatbuffers_builder_t* B) {
    return (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_start_struct(B, 44, 4);
}
static inline FripuckProtocol_Sensors_ImuData_ref_t FripuckProtocol_Sensors_ImuData_end(flatbuffers_builder_t* B) {
    if (!(1)) {
        FripuckProtocol_Sensors_ImuData_to_pe((FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_struct_edit(B));
    }
    return flatcc_builder_end_struct(B);
}
static inline FripuckProtocol_Sensors_ImuData_ref_t FripuckProtocol_Sensors_ImuData_end_pe(flatbuffers_builder_t* B) {
    return flatcc_builder_end_struct(B);
}
static inline FripuckProtocol_Sensors_ImuData_ref_t FripuckProtocol_Sensors_ImuData_create(flatbuffers_builder_t* B,
                                                                                           float v0, float v1, float v2,
                                                                                           float v3, float v4, float v5,
                                                                                           float v6, float v7, float v8,
                                                                                           float v9, uint16_t v10) {
    FripuckProtocol_Sensors_ImuData_t* _p = FripuckProtocol_Sensors_ImuData_start(B);
    if (!_p) return 0;
    FripuckProtocol_Sensors_ImuData_assign_to_pe(_p, v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10);
    return FripuckProtocol_Sensors_ImuData_end_pe(B);
}
static inline FripuckProtocol_Sensors_ImuData_ref_t FripuckProtocol_Sensors_ImuData_clone(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_ImuData_struct_t p) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    do {
        flatcc_builder_ref_t _ref;
        if ((_ref = flatcc_builder_refmap_find((B), (p)))) return _ref;
    } while (0);
    _p = FripuckProtocol_Sensors_ImuData_start(B);
    if (!_p) return 0;
    FripuckProtocol_Sensors_ImuData_copy(_p, p);
    do {
        return flatcc_builder_refmap_insert((B), (p), (FripuckProtocol_Sensors_ImuData_end_pe(B)));
    } while (0);
}
typedef flatbuffers_ref_t FripuckProtocol_Sensors_ImuData_vec_ref_t;
static inline int FripuckProtocol_Sensors_ImuData_vec_start(flatbuffers_builder_t* B) {
    return flatcc_builder_start_vector(B, 44, 4, ((0xffffffffUL) / ((44) == 0 ? 1 : (44))));
}
static inline FripuckProtocol_Sensors_ImuData_vec_ref_t FripuckProtocol_Sensors_ImuData_vec_end_pe(
    flatbuffers_builder_t* B) {
    return flatcc_builder_end_vector(B);
}
static inline FripuckProtocol_Sensors_ImuData_vec_ref_t FripuckProtocol_Sensors_ImuData_vec_end(
    flatbuffers_builder_t* B) {
    if (!(1)) {
        size_t i, n;
        FripuckProtocol_Sensors_ImuData_t* p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_vector_edit(B);
        for (i = 0, n = flatcc_builder_vector_count(B); i < n; ++i) {
            FripuckProtocol_Sensors_ImuData_to_pe(FripuckProtocol_Sensors_ImuData__ptr_add(p, i));
        }
    }
    return flatcc_builder_end_vector(B);
}
static inline FripuckProtocol_Sensors_ImuData_vec_ref_t FripuckProtocol_Sensors_ImuData_vec_create_pe(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* data, size_t len) {
    return flatcc_builder_create_vector(B, data, len, 44, 4, ((0xffffffffUL) / ((44) == 0 ? 1 : (44))));
}
static inline FripuckProtocol_Sensors_ImuData_vec_ref_t FripuckProtocol_Sensors_ImuData_vec_create(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* data, size_t len) {
    if (!(1)) {
        size_t i;
        FripuckProtocol_Sensors_ImuData_t* p;
        int ret = flatcc_builder_start_vector(B, 44, 4, ((0xffffffffUL) / ((44) == 0 ? 1 : (44))));
        if (ret) {
            return ret;
        }
        p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, len);
        if (!p) return 0;
        for (i = 0; i < len; ++i) {
            FripuckProtocol_Sensors_ImuData_copy_to_pe(FripuckProtocol_Sensors_ImuData__ptr_add(p, i),
                                                       FripuckProtocol_Sensors_ImuData__const_ptr_add(data, i));
        }
        return flatcc_builder_end_vector(B);
    } else
        return flatcc_builder_create_vector(B, data, len, 44, 4, ((0xffffffffUL) / ((44) == 0 ? 1 : (44))));
}
static inline FripuckProtocol_Sensors_ImuData_vec_ref_t FripuckProtocol_Sensors_ImuData_vec_clone(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_ImuData_vec_t vec) {
    do {
        do {
            flatcc_builder_ref_t _ref;
            if ((_ref = flatcc_builder_refmap_find((B), (vec)))) return _ref;
        } while (0);
        do {
            return flatcc_builder_refmap_insert(
                (B), (vec),
                (flatcc_builder_create_vector(B, vec, FripuckProtocol_Sensors_ImuData_vec_len(vec), 44, 4,
                                              ((0xffffffffUL) / ((44) == 0 ? 1 : (44))))));
        } while (0);
    } while (0);
}
static inline FripuckProtocol_Sensors_ImuData_vec_ref_t FripuckProtocol_Sensors_ImuData_vec_slice(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_ImuData_vec_t vec, size_t index, size_t len) {
    size_t n = FripuckProtocol_Sensors_ImuData_vec_len(vec);
    if (index >= n) index = n;
    n -= index;
    if (len > n) len = n;
    return flatcc_builder_create_vector(B, FripuckProtocol_Sensors_ImuData__const_ptr_add(vec, index), len, 44, 4,
                                        ((0xffffffffUL) / ((44) == 0 ? 1 : (44))));
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_vec_extend(flatbuffers_builder_t* B,
                                                                                            size_t len) {
    return (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, len);
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_vec_append(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* data, size_t len) {
    return (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_append_vector(B, data, len);
}
static inline int FripuckProtocol_Sensors_ImuData_vec_truncate(flatbuffers_builder_t* B, size_t len) {
    return flatcc_builder_truncate_vector(B, len);
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_vec_edit(flatbuffers_builder_t* B) {
    return (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_vector_edit(B);
}
static inline size_t FripuckProtocol_Sensors_ImuData_vec_reserved_len(flatbuffers_builder_t* B) {
    return flatcc_builder_vector_count(B);
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_vec_push(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* p) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    return (_p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, 1))
               ? (memcpy(_p, p, FripuckProtocol_Sensors_ImuData__size()), _p)
               : 0;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_vec_push_copy(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* p) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    return (_p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, 1))
               ? FripuckProtocol_Sensors_ImuData_copy(_p, p)
               : 0;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_vec_push_clone(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* p) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    return (_p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, 1))
               ? FripuckProtocol_Sensors_ImuData_copy(_p, p)
               : 0;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_vec_push_create(
    flatbuffers_builder_t* B, float v0, float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8,
    float v9, uint16_t v10) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    return (_p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, 1))
               ? FripuckProtocol_Sensors_ImuData_assign(_p, v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10)
               : 0;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_start_as_root(
    flatbuffers_builder_t* B) {
    return flatbuffers_buffer_start(B, "SENS") ? 0 : FripuckProtocol_Sensors_ImuData_start(B);
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_start_as_root_with_size(
    flatbuffers_builder_t* B) {
    return flatbuffers_buffer_start_with_size(B, "SENS") ? 0 : FripuckProtocol_Sensors_ImuData_start(B);
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_start_as_typed_root(
    flatbuffers_builder_t* B) {
    return flatbuffers_buffer_start(B, "\x6f\xc5\x8a\x68") ? 0 : FripuckProtocol_Sensors_ImuData_start(B);
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_ImuData_start_as_typed_root_with_size(
    flatbuffers_builder_t* B) {
    return flatbuffers_buffer_start_with_size(B, "\x6f\xc5\x8a\x68") ? 0 : FripuckProtocol_Sensors_ImuData_start(B);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_ImuData_end_as_root(flatbuffers_builder_t* B) {
    return flatbuffers_buffer_end(B, FripuckProtocol_Sensors_ImuData_end(B));
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_ImuData_end_as_typed_root(flatbuffers_builder_t* B) {
    return flatbuffers_buffer_end(B, FripuckProtocol_Sensors_ImuData_end(B));
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_ImuData_end_pe_as_root(flatbuffers_builder_t* B) {
    return flatbuffers_buffer_end(B, FripuckProtocol_Sensors_ImuData_end_pe(B));
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_ImuData_end_pe_as_typed_root(flatbuffers_builder_t* B) {
    return flatbuffers_buffer_end(B, FripuckProtocol_Sensors_ImuData_end_pe(B));
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_ImuData_create_as_root(flatbuffers_builder_t* B,
                                                                                      float v0, float v1, float v2,
                                                                                      float v3, float v4, float v5,
                                                                                      float v6, float v7, float v8,
                                                                                      float v9, uint16_t v10) {
    return flatcc_builder_create_buffer(
        B, "SENS", 0, FripuckProtocol_Sensors_ImuData_create(B, v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10), 4, 0);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_ImuData_create_as_root_with_size(
    flatbuffers_builder_t* B, float v0, float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8,
    float v9, uint16_t v10) {
    return flatcc_builder_create_buffer(
        B, "SENS", 0, FripuckProtocol_Sensors_ImuData_create(B, v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10), 4,
        flatcc_builder_with_size);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_ImuData_create_as_typed_root(
    flatbuffers_builder_t* B, float v0, float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8,
    float v9, uint16_t v10) {
    return flatcc_builder_create_buffer(
        B, "\x6f\xc5\x8a\x68", 0,
        FripuckProtocol_Sensors_ImuData_create(B, v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10), 4, 0);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_ImuData_create_as_typed_root_with_size(
    flatbuffers_builder_t* B, float v0, float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8,
    float v9, uint16_t v10) {
    return flatcc_builder_create_buffer(
        B, "\x6f\xc5\x8a\x68", 0,
        FripuckProtocol_Sensors_ImuData_create(B, v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10), 4,
        flatcc_builder_with_size);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_ImuData_clone_as_root(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_ImuData_struct_t p) {
    return flatcc_builder_create_buffer(B, "SENS", 0, FripuckProtocol_Sensors_ImuData_clone(B, p), 4, 0);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_ImuData_clone_as_root_with_size(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_ImuData_struct_t p) {
    return flatcc_builder_create_buffer(B, "SENS", 0, FripuckProtocol_Sensors_ImuData_clone(B, p), 4,
                                        flatcc_builder_with_size);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_ImuData_clone_as_typed_root(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_ImuData_struct_t p) {
    return flatcc_builder_create_buffer(B, "\x6f\xc5\x8a\x68", 0, FripuckProtocol_Sensors_ImuData_clone(B, p), 4, 0);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_ImuData_clone_as_typed_root_with_size(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_ImuData_struct_t p) {
    return flatcc_builder_create_buffer(B, "\x6f\xc5\x8a\x68", 0, FripuckProtocol_Sensors_ImuData_clone(B, p), 4,
                                        flatcc_builder_with_size);
}

/////// tof ////////////
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_to_pe(
    FripuckProtocol_Sensors_TofData_t* p) {
    if (!(1)) {
        FripuckProtocol_Sensors_TofData_copy_to_pe(p, p);
    };
    return p;
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_from_pe(
    FripuckProtocol_Sensors_TofData_t* p) {
    if (!(1)) {
        FripuckProtocol_Sensors_TofData_copy_from_pe(p, p);
    };
    return p;
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_clear(
    FripuckProtocol_Sensors_TofData_t* p) {
    return (FripuckProtocol_Sensors_TofData_t*)memset(p, 0, FripuckProtocol_Sensors_TofData__size());
}
typedef flatbuffers_ref_t FripuckProtocol_Sensors_TofData_ref_t;
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_start(flatbuffers_builder_t* B) {
    return (FripuckProtocol_Sensors_TofData_t*)flatcc_builder_start_struct(B, 4, 2);
}
static inline FripuckProtocol_Sensors_TofData_ref_t FripuckProtocol_Sensors_TofData_end(flatbuffers_builder_t* B) {
    if (!(1)) {
        FripuckProtocol_Sensors_TofData_to_pe((FripuckProtocol_Sensors_TofData_t*)flatcc_builder_struct_edit(B));
    }
    return flatcc_builder_end_struct(B);
}
static inline FripuckProtocol_Sensors_TofData_ref_t FripuckProtocol_Sensors_TofData_end_pe(flatbuffers_builder_t* B) {
    return flatcc_builder_end_struct(B);
}
static inline FripuckProtocol_Sensors_TofData_ref_t FripuckProtocol_Sensors_TofData_create(flatbuffers_builder_t* B,
                                                                                           uint16_t v0, uint16_t v1) {
    FripuckProtocol_Sensors_TofData_t* _p = FripuckProtocol_Sensors_TofData_start(B);
    if (!_p) return 0;
    FripuckProtocol_Sensors_TofData_assign_to_pe(_p, v0, v1);
    return FripuckProtocol_Sensors_TofData_end_pe(B);
}
static inline FripuckProtocol_Sensors_TofData_ref_t FripuckProtocol_Sensors_TofData_clone(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_TofData_struct_t p) {
    FripuckProtocol_Sensors_TofData_t* _p;
    do {
        flatcc_builder_ref_t _ref;
        if ((_ref = flatcc_builder_refmap_find((B), (p)))) return _ref;
    } while (0);
    _p = FripuckProtocol_Sensors_TofData_start(B);
    if (!_p) return 0;
    FripuckProtocol_Sensors_TofData_copy(_p, p);
    do {
        return flatcc_builder_refmap_insert((B), (p), (FripuckProtocol_Sensors_TofData_end_pe(B)));
    } while (0);
}
typedef flatbuffers_ref_t FripuckProtocol_Sensors_TofData_vec_ref_t;
static inline int FripuckProtocol_Sensors_TofData_vec_start(flatbuffers_builder_t* B) {
    return flatcc_builder_start_vector(B, 4, 2, ((0xffffffffUL) / ((4) == 0 ? 1 : (4))));
}
static inline FripuckProtocol_Sensors_TofData_vec_ref_t FripuckProtocol_Sensors_TofData_vec_end_pe(
    flatbuffers_builder_t* B) {
    return flatcc_builder_end_vector(B);
}
static inline FripuckProtocol_Sensors_TofData_vec_ref_t FripuckProtocol_Sensors_TofData_vec_end(
    flatbuffers_builder_t* B) {
    if (!(1)) {
        size_t i, n;
        FripuckProtocol_Sensors_TofData_t* p = (FripuckProtocol_Sensors_TofData_t*)flatcc_builder_vector_edit(B);
        for (i = 0, n = flatcc_builder_vector_count(B); i < n; ++i) {
            FripuckProtocol_Sensors_TofData_to_pe(FripuckProtocol_Sensors_TofData__ptr_add(p, i));
        }
    }
    return flatcc_builder_end_vector(B);
}
static inline FripuckProtocol_Sensors_TofData_vec_ref_t FripuckProtocol_Sensors_TofData_vec_create_pe(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_TofData_t* data, size_t len) {
    return flatcc_builder_create_vector(B, data, len, 4, 2, ((0xffffffffUL) / ((4) == 0 ? 1 : (4))));
}
static inline FripuckProtocol_Sensors_TofData_vec_ref_t FripuckProtocol_Sensors_TofData_vec_create(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_TofData_t* data, size_t len) {
    if (!(1)) {
        size_t i;
        FripuckProtocol_Sensors_TofData_t* p;
        int ret = flatcc_builder_start_vector(B, 4, 2, ((0xffffffffUL) / ((4) == 0 ? 1 : (4))));
        if (ret) {
            return ret;
        }
        p = (FripuckProtocol_Sensors_TofData_t*)flatcc_builder_extend_vector(B, len);
        if (!p) return 0;
        for (i = 0; i < len; ++i) {
            FripuckProtocol_Sensors_TofData_copy_to_pe(FripuckProtocol_Sensors_TofData__ptr_add(p, i),
                                                       FripuckProtocol_Sensors_TofData__const_ptr_add(data, i));
        }
        return flatcc_builder_end_vector(B);
    } else
        return flatcc_builder_create_vector(B, data, len, 4, 2, ((0xffffffffUL) / ((4) == 0 ? 1 : (4))));
}
static inline FripuckProtocol_Sensors_TofData_vec_ref_t FripuckProtocol_Sensors_TofData_vec_clone(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_TofData_vec_t vec) {
    do {
        do {
            flatcc_builder_ref_t _ref;
            if ((_ref = flatcc_builder_refmap_find((B), (vec)))) return _ref;
        } while (0);
        do {
            return flatcc_builder_refmap_insert(
                (B), (vec),
                (flatcc_builder_create_vector(B, vec, FripuckProtocol_Sensors_TofData_vec_len(vec), 4, 2,
                                              ((0xffffffffUL) / ((4) == 0 ? 1 : (4))))));
        } while (0);
    } while (0);
}
static inline FripuckProtocol_Sensors_TofData_vec_ref_t FripuckProtocol_Sensors_TofData_vec_slice(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_TofData_vec_t vec, size_t index, size_t len) {
    size_t n = FripuckProtocol_Sensors_TofData_vec_len(vec);
    if (index >= n) index = n;
    n -= index;
    if (len > n) len = n;
    return flatcc_builder_create_vector(B, FripuckProtocol_Sensors_TofData__const_ptr_add(vec, index), len, 4, 2,
                                        ((0xffffffffUL) / ((4) == 0 ? 1 : (4))));
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_vec_extend(flatbuffers_builder_t* B,
                                                                                            size_t len) {
    return (FripuckProtocol_Sensors_TofData_t*)flatcc_builder_extend_vector(B, len);
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_vec_append(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_TofData_t* data, size_t len) {
    return (FripuckProtocol_Sensors_TofData_t*)flatcc_builder_append_vector(B, data, len);
}
static inline int FripuckProtocol_Sensors_TofData_vec_truncate(flatbuffers_builder_t* B, size_t len) {
    return flatcc_builder_truncate_vector(B, len);
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_vec_edit(flatbuffers_builder_t* B) {
    return (FripuckProtocol_Sensors_TofData_t*)flatcc_builder_vector_edit(B);
}
static inline size_t FripuckProtocol_Sensors_TofData_vec_reserved_len(flatbuffers_builder_t* B) {
    return flatcc_builder_vector_count(B);
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_vec_push(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_TofData_t* p) {
    FripuckProtocol_Sensors_TofData_t* _p;
    return (_p = (FripuckProtocol_Sensors_TofData_t*)flatcc_builder_extend_vector(B, 1))
               ? (memcpy(_p, p, FripuckProtocol_Sensors_TofData__size()), _p)
               : 0;
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_vec_push_copy(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_TofData_t* p) {
    FripuckProtocol_Sensors_TofData_t* _p;
    return (_p = (FripuckProtocol_Sensors_TofData_t*)flatcc_builder_extend_vector(B, 1))
               ? FripuckProtocol_Sensors_TofData_copy(_p, p)
               : 0;
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_vec_push_clone(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_TofData_t* p) {
    FripuckProtocol_Sensors_TofData_t* _p;
    return (_p = (FripuckProtocol_Sensors_TofData_t*)flatcc_builder_extend_vector(B, 1))
               ? FripuckProtocol_Sensors_TofData_copy(_p, p)
               : 0;
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_vec_push_create(
    flatbuffers_builder_t* B, uint16_t v0, uint16_t v1) {
    FripuckProtocol_Sensors_TofData_t* _p;
    return (_p = (FripuckProtocol_Sensors_TofData_t*)flatcc_builder_extend_vector(B, 1))
               ? FripuckProtocol_Sensors_TofData_assign(_p, v0, v1)
               : 0;
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_start_as_root(
    flatbuffers_builder_t* B) {
    return flatbuffers_buffer_start(B, "SENS") ? 0 : FripuckProtocol_Sensors_TofData_start(B);
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_start_as_root_with_size(
    flatbuffers_builder_t* B) {
    return flatbuffers_buffer_start_with_size(B, "SENS") ? 0 : FripuckProtocol_Sensors_TofData_start(B);
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_start_as_typed_root(
    flatbuffers_builder_t* B) {
    return flatbuffers_buffer_start(B, "\x67\xbb\xa5\x59") ? 0 : FripuckProtocol_Sensors_TofData_start(B);
}
static inline FripuckProtocol_Sensors_TofData_t* FripuckProtocol_Sensors_TofData_start_as_typed_root_with_size(
    flatbuffers_builder_t* B) {
    return flatbuffers_buffer_start_with_size(B, "\x67\xbb\xa5\x59") ? 0 : FripuckProtocol_Sensors_TofData_start(B);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_TofData_end_as_root(flatbuffers_builder_t* B) {
    return flatbuffers_buffer_end(B, FripuckProtocol_Sensors_TofData_end(B));
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_TofData_end_as_typed_root(flatbuffers_builder_t* B) {
    return flatbuffers_buffer_end(B, FripuckProtocol_Sensors_TofData_end(B));
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_TofData_end_pe_as_root(flatbuffers_builder_t* B) {
    return flatbuffers_buffer_end(B, FripuckProtocol_Sensors_TofData_end_pe(B));
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_TofData_end_pe_as_typed_root(flatbuffers_builder_t* B) {
    return flatbuffers_buffer_end(B, FripuckProtocol_Sensors_TofData_end_pe(B));
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_TofData_create_as_root(flatbuffers_builder_t* B,
                                                                                      uint16_t v0, uint16_t v1) {
    return flatcc_builder_create_buffer(B, "SENS", 0, FripuckProtocol_Sensors_TofData_create(B, v0, v1), 2, 0);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_TofData_create_as_root_with_size(
    flatbuffers_builder_t* B, uint16_t v0, uint16_t v1) {
    return flatcc_builder_create_buffer(B, "SENS", 0, FripuckProtocol_Sensors_TofData_create(B, v0, v1), 2,
                                        flatcc_builder_with_size);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_TofData_create_as_typed_root(flatbuffers_builder_t* B,
                                                                                            uint16_t v0, uint16_t v1) {
    return flatcc_builder_create_buffer(B, "\x67\xbb\xa5\x59", 0, FripuckProtocol_Sensors_TofData_create(B, v0, v1), 2,
                                        0);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_TofData_create_as_typed_root_with_size(
    flatbuffers_builder_t* B, uint16_t v0, uint16_t v1) {
    return flatcc_builder_create_buffer(B, "\x67\xbb\xa5\x59", 0, FripuckProtocol_Sensors_TofData_create(B, v0, v1), 2,
                                        flatcc_builder_with_size);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_TofData_clone_as_root(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_TofData_struct_t p) {
    return flatcc_builder_create_buffer(B, "SENS", 0, FripuckProtocol_Sensors_TofData_clone(B, p), 2, 0);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_TofData_clone_as_root_with_size(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_TofData_struct_t p) {
    return flatcc_builder_create_buffer(B, "SENS", 0, FripuckProtocol_Sensors_TofData_clone(B, p), 2,
                                        flatcc_builder_with_size);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_TofData_clone_as_typed_root(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_TofData_struct_t p) {
    return flatcc_builder_create_buffer(B, "\x67\xbb\xa5\x59", 0, FripuckProtocol_Sensors_TofData_clone(B, p), 2, 0);
}
static inline flatbuffers_buffer_ref_t FripuckProtocol_Sensors_TofData_clone_as_typed_root_with_size(
    flatbuffers_builder_t* B, FripuckProtocol_Sensors_TofData_struct_t p) {
    return flatcc_builder_create_buffer(B, "\x67\xbb\xa5\x59", 0, FripuckProtocol_Sensors_TofData_clone(B, p), 2,
                                        flatcc_builder_with_size);
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_add(flatbuffers_builder_t* B,
                                                              FripuckProtocol_Sensors_ImuData_vec_ref_t ref) {
    FripuckProtocol_Sensors_ImuData_vec_ref_t* _p;
    return (ref && (_p = flatcc_builder_table_add_offset(B, 6))) ? ((*_p = ref), 0) : -1;
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_start(flatbuffers_builder_t* B) {
    return FripuckProtocol_Sensors_ImuData_vec_start(B);
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_end_pe(flatbuffers_builder_t* B) {
    return FripuckProtocol_Sensors_SensorBatch_imu_add(B, FripuckProtocol_Sensors_ImuData_vec_end_pe(B));
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_end(flatbuffers_builder_t* B) {
    return FripuckProtocol_Sensors_SensorBatch_imu_add(B, FripuckProtocol_Sensors_ImuData_vec_end(B));
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_create_pe(flatbuffers_builder_t* B,
                                                                    const FripuckProtocol_Sensors_ImuData_t* data,
                                                                    size_t len) {
    return FripuckProtocol_Sensors_SensorBatch_imu_add(B, FripuckProtocol_Sensors_ImuData_vec_create_pe(B, data, len));
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_create(flatbuffers_builder_t* B,
                                                                 const FripuckProtocol_Sensors_ImuData_t* data,
                                                                 size_t len) {
    return FripuckProtocol_Sensors_SensorBatch_imu_add(B, FripuckProtocol_Sensors_ImuData_vec_create(B, data, len));
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_slice(flatbuffers_builder_t* B,
                                                                FripuckProtocol_Sensors_ImuData_vec_t vec, size_t index,
                                                                size_t len) {
    return FripuckProtocol_Sensors_SensorBatch_imu_add(B,
                                                       FripuckProtocol_Sensors_ImuData_vec_slice(B, vec, index, len));
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_clone(flatbuffers_builder_t* B,
                                                                FripuckProtocol_Sensors_ImuData_vec_t vec) {
    return FripuckProtocol_Sensors_SensorBatch_imu_add(B, FripuckProtocol_Sensors_ImuData_vec_clone(B, vec));
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_pick(flatbuffers_builder_t* B,
                                                               FripuckProtocol_Sensors_SensorBatch_table_t t) {
    FripuckProtocol_Sensors_ImuData_vec_t _p = FripuckProtocol_Sensors_SensorBatch_imu_get(t);
    return _p ? FripuckProtocol_Sensors_SensorBatch_imu_clone(B, _p) : 0;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_extend(
    flatbuffers_builder_t* B, size_t len) {
    return (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, len);
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_append(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* data, size_t len) {
    return (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_append_vector(B, data, len);
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_truncate(flatbuffers_builder_t* B, size_t len) {
    return flatcc_builder_truncate_vector(B, len);
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_edit(
    flatbuffers_builder_t* B) {
    return (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_vector_edit(B);
}
static inline size_t FripuckProtocol_Sensors_SensorBatch_imu_reserved_len(flatbuffers_builder_t* B) {
    return flatcc_builder_vector_count(B);
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_push(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* p) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    return (_p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, 1))
               ? (memcpy(_p, p, FripuckProtocol_Sensors_ImuData__size()), _p)
               : 0;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_push_copy(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* p) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    return (_p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, 1))
               ? FripuckProtocol_Sensors_ImuData_copy(_p, p)
               : 0;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_push_clone(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* p) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    return (_p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, 1))
               ? FripuckProtocol_Sensors_ImuData_copy(_p, p)
               : 0;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_push_create(
    flatbuffers_builder_t* B, float v0, float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8,
    float v9, uint16_t v10) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    return (_p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, 1))
               ? FripuckProtocol_Sensors_ImuData_assign(_p, v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10)
               : 0;
}

static inline int FripuckProtocol_Sensors_SensorBatch_imu_add(flatbuffers_builder_t* B,
                                                              FripuckProtocol_Sensors_ImuData_vec_ref_t ref) {
    FripuckProtocol_Sensors_ImuData_vec_ref_t* _p;
    return (ref && (_p = flatcc_builder_table_add_offset(B, 6))) ? ((*_p = ref), 0) : -1;
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_start(flatbuffers_builder_t* B) {
    return FripuckProtocol_Sensors_ImuData_vec_start(B);
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_end_pe(flatbuffers_builder_t* B) {
    return FripuckProtocol_Sensors_SensorBatch_imu_add(B, FripuckProtocol_Sensors_ImuData_vec_end_pe(B));
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_end(flatbuffers_builder_t* B) {
    return FripuckProtocol_Sensors_SensorBatch_imu_add(B, FripuckProtocol_Sensors_ImuData_vec_end(B));
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_create_pe(flatbuffers_builder_t* B,
                                                                    const FripuckProtocol_Sensors_ImuData_t* data,
                                                                    size_t len) {
    return FripuckProtocol_Sensors_SensorBatch_imu_add(B, FripuckProtocol_Sensors_ImuData_vec_create_pe(B, data, len));
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_create(flatbuffers_builder_t* B,
                                                                 const FripuckProtocol_Sensors_ImuData_t* data,
                                                                 size_t len) {
    return FripuckProtocol_Sensors_SensorBatch_imu_add(B, FripuckProtocol_Sensors_ImuData_vec_create(B, data, len));
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_slice(flatbuffers_builder_t* B,
                                                                FripuckProtocol_Sensors_ImuData_vec_t vec, size_t index,
                                                                size_t len) {
    return FripuckProtocol_Sensors_SensorBatch_imu_add(B,
                                                       FripuckProtocol_Sensors_ImuData_vec_slice(B, vec, index, len));
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_clone(flatbuffers_builder_t* B,
                                                                FripuckProtocol_Sensors_ImuData_vec_t vec) {
    return FripuckProtocol_Sensors_SensorBatch_imu_add(B, FripuckProtocol_Sensors_ImuData_vec_clone(B, vec));
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_pick(flatbuffers_builder_t* B,
                                                               FripuckProtocol_Sensors_SensorBatch_table_t t) {
    FripuckProtocol_Sensors_ImuData_vec_t _p = FripuckProtocol_Sensors_SensorBatch_imu_get(t);
    return _p ? FripuckProtocol_Sensors_SensorBatch_imu_clone(B, _p) : 0;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_extend(
    flatbuffers_builder_t* B, size_t len) {
    return (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, len);
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_append(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* data, size_t len) {
    return (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_append_vector(B, data, len);
}
static inline int FripuckProtocol_Sensors_SensorBatch_imu_truncate(flatbuffers_builder_t* B, size_t len) {
    return flatcc_builder_truncate_vector(B, len);
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_edit(
    flatbuffers_builder_t* B) {
    return (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_vector_edit(B);
}
static inline size_t FripuckProtocol_Sensors_SensorBatch_imu_reserved_len(flatbuffers_builder_t* B) {
    return flatcc_builder_vector_count(B);
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_push(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* p) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    return (_p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, 1))
               ? (memcpy(_p, p, FripuckProtocol_Sensors_ImuData__size()), _p)
               : 0;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_push_copy(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* p) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    return (_p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, 1))
               ? FripuckProtocol_Sensors_ImuData_copy(_p, p)
               : 0;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_push_clone(
    flatbuffers_builder_t* B, const FripuckProtocol_Sensors_ImuData_t* p) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    return (_p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, 1))
               ? FripuckProtocol_Sensors_ImuData_copy(_p, p)
               : 0;
}
static inline FripuckProtocol_Sensors_ImuData_t* FripuckProtocol_Sensors_SensorBatch_imu_push_create(
    flatbuffers_builder_t* B, float v0, float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8,
    float v9, uint16_t v10) {
    FripuckProtocol_Sensors_ImuData_t* _p;
    return (_p = (FripuckProtocol_Sensors_ImuData_t*)flatcc_builder_extend_vector(B, 1))
               ? FripuckProtocol_Sensors_ImuData_assign(_p, v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10)
               : 0;
}