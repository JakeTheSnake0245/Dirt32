/*
 * chacha20poly1305.h — RFC 8439 ChaCha20-Poly1305 AEAD.
 * Portable C99, constant-time tag compare, no dependencies.
 */
#ifndef SPS_CHACHA20POLY1305_H
#define SPS_CHACHA20POLY1305_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Encrypt-and-tag. tag is 16 bytes. ct may alias pt.
 */
void cc20p1305_encrypt(const uint8_t key[32], const uint8_t nonce[12],
                       const uint8_t *aad, size_t aad_len,
                       const uint8_t *pt, size_t pt_len,
                       uint8_t *ct, uint8_t tag[16]);

/*
 * Verify-then-decrypt. Returns 0 on success, -1 on auth failure
 * (in which case pt is not written). pt may alias ct.
 */
int cc20p1305_decrypt(const uint8_t key[32], const uint8_t nonce[12],
                      const uint8_t *aad, size_t aad_len,
                      const uint8_t *ct, size_t ct_len,
                      const uint8_t tag[16], uint8_t *pt);

#ifdef __cplusplus
}
#endif
#endif
