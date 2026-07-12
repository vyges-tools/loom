//! Clean-room LZ4 **block** decompressor (no framing), enough to inflate FST
//! `HIER_LZ4` hierarchy blocks. LZ4 block format: a series of sequences, each a
//! token byte (`literal_len<<4 | match_len`), optional length-extension bytes, the
//! literal bytes, then a 2-byte little-endian back-offset and a (possibly extended)
//! match length; `+4` minmatch. The final sequence is literals only. Overlapping
//! copies are byte-by-byte. `expected` is the known uncompressed length (FST stores it).

pub(crate) fn decompress(input: &[u8], expected: usize) -> Result<Vec<u8>, String> {
    let mut out: Vec<u8> = Vec::with_capacity(expected);
    let mut i = 0usize;

    let ext = |i: &mut usize| -> Result<usize, String> {
        let mut n = 0usize;
        loop {
            let b = *input.get(*i).ok_or("lz4: length overrun")?;
            *i += 1;
            n += b as usize;
            if b != 255 {
                return Ok(n);
            }
        }
    };

    while i < input.len() {
        let token = input[i];
        i += 1;

        // literals
        let mut lit = (token >> 4) as usize;
        if lit == 15 {
            lit += ext(&mut i)?;
        }
        let end = i.checked_add(lit).ok_or("lz4: literal overflow")?;
        if end > input.len() {
            return Err("lz4: literal copy overrun".into());
        }
        out.extend_from_slice(&input[i..end]);
        i = end;
        if i >= input.len() {
            break; // final sequence: literals only
        }

        // match
        if i + 2 > input.len() {
            return Err("lz4: missing offset".into());
        }
        let offset = (input[i] as usize) | ((input[i + 1] as usize) << 8);
        i += 2;
        if offset == 0 || offset > out.len() {
            return Err("lz4: bad match offset".into());
        }
        let mut mlen = (token & 0x0f) as usize;
        if mlen == 15 {
            mlen += ext(&mut i)?;
        }
        mlen += 4; // minmatch

        let mut src = out.len() - offset;
        for _ in 0..mlen {
            let b = out[src];
            out.push(b);
            src += 1;
        }
    }

    if out.len() != expected {
        return Err(format!("lz4: got {} bytes, expected {}", out.len(), expected));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Round-trip against a tiny hand-built LZ4 block: "aaaaaaaa" (8 'a').
    // token 0x11 (lit=1, match=1) | literal 'a' | offset=1 LE | (match=1+4=5) -> 1+5=6? build carefully.
    #[test]
    fn literal_then_overlap_run() {
        // "abcabcabc": literal "abc" (lit=3), then match offset=3 len=6 (3+... ) -> minmatch4+2
        // token: lit=3 -> hi nibble 3; match extra: want copy 6 -> mlen field = 6-4 = 2 -> lo nibble 2
        // token = 0x32; literals b"abc"; offset=3 (0x03 0x00); no extension.
        let comp = [0x32u8, b'a', b'b', b'c', 0x03, 0x00];
        let out = decompress(&comp, 9).unwrap();
        assert_eq!(&out, b"abcabcabc");
    }

    #[test]
    fn all_literals() {
        // token lit=5, match nibble ignored (final seq literals only), 5 literal bytes.
        let comp = [0x50u8, b'h', b'e', b'l', b'l', b'o'];
        let out = decompress(&comp, 5).unwrap();
        assert_eq!(&out, b"hello");
    }
}
