//! Native pure-Rust VAG ODIS `.frf` / `.odx` container reader and Continental SIMOS18.1 unpacker.
//!
//! Unpacks official VAG ODIS flash containers (`FL_*.frf` or `.odx`) directly into a 4 MB
//! SIMOS18.1 PFlash image (`0x400000` bytes) in memory without Python scripts or external tools:
//!
//! 1. **FRF container stream cipher (`FRF -> ZIP`)**: 4095-byte key table with two 8-bit rolling
//!    state accumulators (`bri3d/VW_Flash`, BSD-2-Clause), yielding a standard `PK\x03\x04` ZIP
//!    archive whose single entry is `<name>.odx`.
//! 2. **Safe ZIP Central Directory + RFC 1951 DEFLATE (`ZIP -> ODX`)**: bounds-checked Central
//!    Directory reader paired with `miniz_oxide` (`decompress_to_vec_with_limit`) and IEEE CRC-32
//!    verification against zip-bombs and corrupt archives.
//! 3. **SIMOS18.1 CBOOT `0xAA` (`AES-128-CBC` + `LZSS`) block pipeline** (`VERIFIED-S` in
//!    `8V0906264H__0005` CBOOT, REA `ev_6a15cf31c84ad388f965876ff52e65a7b0971414575c0dc4fd70258a7b3b72ba`,
//!    `ev_ce0940cdbb9cf30dc0d50ea2fb6fe5d2045c8b92b26cd881bcb253f484d0a53b`,
//!    `ev_18046840217e4e3be97e205fc8b12795e92e1aec45b179f2cb345fce584e4fd4`,
//!    `ev_aff0db1293c9a949cf3b04d261f112411a5aaa3ba08a5b8ee34d074db05a9875`,
//!    `ev_e23f7de92d811693d93f487817ce6dffb61f85ae4d3237fd6b39ac7f04623aa3`,
//!    `ev_8ceed2cd105da3a9bb1c4f987cafd838be53343b62fc244f82d1a7fe06f5e48b`):
//!    - Encryption `'A'`: AES-128-CBC with CBOOT key @ `0x800220C0` (`0xD00080C0`)
//!      `98D31202E48E3854F2CA561545BA6F2F` and IV @ `0x800220B0` (`0xD00080B0`)
//!      `E7861278C508532798BCA4FE451D20D1` (`0x8002F134..0x8002F144`).
//!    - Compression `'A'`: CBOOT LZSS (`0x8002EC94..0x8002F0B6`), initialized at `0x8002F1FA` with
//!      a zero-filled `0x3FF`-byte (1023-byte) ring buffer (`win_pos = 0`), MSB-first flag bytes
//!      (`0` = literal byte, `1` = 2-byte `(b0, b1)` back-reference with `len = b0 >> 2` and
//!      `offset = ((b0 & 3) << 8) | b1`, valid iff `len <= offset <= 0x3FF`), stopping at the
//!      block's exact `<UNCOMPRESSED-SIZE>`.
//! 4. **Post-unpack integrity check**: verifies all present CBOOT block CRC-32 headers and ECM3
//!    sums via [`crate::verify_checksums`].

use crate::checksum::verify_checksums;
use crate::post_programming::{apply_post_programming, dq250_crc_register};
use serde::Serialize;
use std::collections::HashMap;
use std::path::Path;

/// Full SIMOS18.1 PFlash image size (4 MiB: 2 MiB `PFlash0` + 2 MiB `PFlash1`).
pub const SIMOS18_FLASH_LEN: usize = 0x40_0000;

/// Length of the SIMOS18.1 `SBOOT` + shared supplier ROM region (`0x00000..0x1C000`, 112 KiB).
pub const SIMOS18_SBOOT_LEN: usize = 0x01_C000;

/// Maximum `.frf` container size accepted into memory (32 MiB; real SIMOS18 `.frf` files are ~2.2 MiB).
const MAX_FRF_BYTES: usize = 32 * 1024 * 1024;

/// Maximum inflated `.odx` XML size accepted from the `.frf` ZIP container (64 MiB; real `.odx` files are ~4 MiB).
const MAX_ODX_BYTES: usize = 64 * 1024 * 1024;

/// SIMOS18.1 CBOOT LZSS sliding-window ring length (`0x3FF` = 1023 bytes, `0x8002F1FA`).
const CBOOT_LZSS_WIN_LEN: usize = 0x03FF;

/// SIMOS18.1 CBOOT AES-128-CBC key (`0x800220C0` / `0xD00080C0`, `VERIFIED-S`).
const S18_AES_KEY: [u8; 16] = decode_hex(b"98D31202E48E3854F2CA561545BA6F2F");

/// SIMOS18.1 CBOOT AES-128-CBC IV (`0x800220B0` / `0xD00080B0`, `VERIFIED-S`).
const S18_AES_IV: [u8; 16] = decode_hex(b"E7861278C508532798BCA4FE451D20D1");

/// SIMOS18.1 logical programmable blocks (`SOURCE-START-ADDRESS` `1..=5` / `FD_0..FD_4`):
/// `(block_num, name, file_offset, canonical_size)`.
pub const SIMOS18_FRF_BLOCKS: [(u8, &str, usize, usize); 5] = [
    (1, "CBOOT", 0x01_C000, 0x02_3E00),
    (2, "ASW1", 0x04_0000, 0x0F_FC00),
    (3, "ASW2", 0x14_0000, 0x0B_FC00),
    (4, "ASW3", 0x28_0000, 0x07_FC00),
    (5, "CAL", 0x20_0000, 0x07_FC00),
];

/// Summary of an unpacked `.frf` / `.odx` container.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct FrfUnpackReport {
    /// Inner `.odx` entry name.
    pub odx_name: String,
    /// SIMOS18 logical block names written (`CBOOT`, `ASW1`, `ASW2`, `ASW3`, `CAL`).
    pub blocks_written: Vec<&'static str>,
    /// 6-character Continental software box code at `CAL+0x02` (e.g. `SC8O40`, `SC8S50`).
    pub box_code: String,
    /// VAG spare part number extracted from `CAL` header (e.g. `8V0906264H`).
    pub part_number: String,
    /// Whether `SBOOT` (`0x00000..0x1C000`) was supplied from a 4 MB bench donor dump.
    pub has_sboot: bool,
}

const fn hex_nibble(b: u8) -> u8 {
    match b {
        b'0'..=b'9' => b - b'0',
        b'a'..=b'f' => b - b'a' + 10,
        b'A'..=b'F' => b - b'A' + 10,
        _ => panic!("invalid hex digit in const table"),
    }
}

const fn decode_hex<const N: usize>(hex: &[u8]) -> [u8; N] {
    assert!(hex.len() == N * 2, "hex slice length mismatch");
    let mut out = [0u8; N];
    let mut i = 0;
    while i < N {
        out[i] = (hex_nibble(hex[2 * i]) << 4) | hex_nibble(hex[2 * i + 1]);
        i += 1;
    }
    out
}

/// 4095-byte VAG `.frf` stream cipher key table (`bri3d/VW_Flash`, BSD-2-Clause).
const FRF_KEY: [u8; 4095] = decode_hex(
    b"3221602d1719426b710f11297c61084375656e727e451b43430e39475e03480c\
      7837644d315b4b6642541a0a61115738661b346b2b6d12123807453f203e1723\
      26560f3a245273432e0b0b7372286a626d6566161f2d376b067d73710c6b6a04\
      25685c3827426a7b5d6d4b755602403303084a0502731a24346a4918154d4959\
      4d771c350240437171467a333d6e222006257f413e6c232b38132a5f0244286f\
      754460620a76375a702168745b7278701e456d127c203b473b7f0c7b596f0c46\
      581108686b2c7c01037d58192a6c726f7c3f64184842653a2e6827130679683a\
      186f44270b214d76211e5d68215c2b2b14121d5b33625d48213b2f7a567e0272\
      5b5f44333d0f7b013c181269430870011d51507165704c323e4d3a1645623c09\
      44201316290e342d36341e2379376a7a7f452463453359016f6571703c6f3f06\
      3a795758366820773b2a1e2c32411d223642383a752d3c1b4d7c0f246b337124\
      3439024e63660467745d6342421c726e6c51782e7c6f6b294b027b631f40536c\
      17601b58492f416e296079252d23024c2e7a2b0812481703282b3d63530c3279\
      6b752b3b60223e75205a683b3b542245655a307d6047102b215c0a4f42193377\
      1f3b61716803622274396e365871597a390d0d055e03381e0960425a080d0205\
      0751314328181f7756732464565f5a131e4a09746b5363470e52723d06184e5b\
      547813200b061267271e470f1f280c583e1b0b102e4079705639024166417f42\
      2f356f4e3954253a0f433e7d5e5356133e077c235051625f091d592b4d107d02\
      40122d16590a3f37386871763969220a49164273780f075d50066146650e3163\
      2e163640133047284718484458715706055146494f4e31537d01595642012d22\
      497014114f275367582b2e64735c4f1a3f706c7d2e20253909212b4835562743\
      34445a786e463e32027b05781b2d316929262921667f2a332905626d191b3a5e\
      1a5973160c79742d357462077b687255573d7b5f06085002022f4c74304f2e50\
      5b5a274e610532512c6124286447040d637a1c726a08507e12712f52050b4810\
      6d253f1e2d6b2027500a72633e141019362534627847552f4213330d7e193028\
      3a02610e587465360736754172153d7f37066a351a0c051e7b5c7f2f05400d1f\
      293936274a6829073a2e5549661334494d6467753720085223163c400147077e\
      211165706c0f1620523b390403551c7f62095329364c0753230711465a77474c\
      0c5316722430520b3523497831251e275d3b555a01562a2a6477274b77187511\
      514873335b1505504d2e2f2f5949624b2543150f7f06185e4d30255642723956\
      5835213d7a045875261031610a0f7224693c5018267a7745793470224c392209\
      654b18674672043851170433304d2541747226347c777e36197c1f7f6f207d4c\
      1f174a0c237d045c7969323503466b642e5e173b52387a0759246e4224146f09\
      2a2d5d50621e7e54643143514b1f4c755d292d156e64732001556251313d0128\
      4f36591a4d7f79085a153d6e4f03505d4a7a702b18247169593605155e453a11\
      565a44542a487d60253d2a735818023d7a67631a207d4a28705e767551232c07\
      402865422113440c52114a2e06634d7d511a263b7f1e2b382a755d3c0c426229\
      110c355d30411c567c7b5a197601265426115d426a5d744f0d53307d1b211906\
      74792c432011504c626f0b611063760821556e7808400d36417f587e29483a64\
      1276323b170b48362d75454d7b1023626c6d432502525e356d013e085c3d2e0c\
      130b2f0e3d366c5c041570266010172a2c6142117f184f13396249625c0a5b46\
      1e400b043c1a250510587434666c38280c3a5503281c491a4e29605652372b5a\
      5b1e2e65393f597a79453a3f362b7024327f634425653210545e6d2a664a050f\
      732c795f2e71046665280f775526654638541d1d7b743e720a57273f4d502e0e\
      726a08536e56697f3e276b52734234716d0f533867756c513406150647767f53\
      79525940076e716e5a201d6f0c2b595a277e311f301f621964623c62415e4a6b\
      4821354c0223665841796b75284b1c2b6d087e7a5e5867712253637c4270577e\
      6840641f665c0e68791c1e0d4f094409481501707a0a44017741547b52146d33\
      5f152e623b0133430c6f7e5e0a4d1b1e3f0d032a0a1c42726a14550879325632\
      370a5b3d09791b14025c5211607f67125a594c501976276c04342f4b40325824\
      77063257592e0e016149644d5a08710d215f24092c3b164b092a6b2e7c3c3128\
      717c593207563233207a3b7f4e01361d08537d4d224819497c0d114c794a5132\
      016a1d6c3a507f485d02573a5f35553c20550344141d06732264210f493a7a33\
      0933214501034e29554b6a34523351645539574e684909572f0d352703062c5b\
      5c5c467a7472272b293e412523581a6638504f280c455a117d33102a3e0d6d12\
      410c14745b461236230263731b4b372c1243763a483a0107547f3b7e43764840\
      026b14197e2617320940580835301c2a39516d645006237b193d0d5a6a424d65\
      224d52243b3f07241f294a1a3d0e204a5b6a27272e724c3c281e3d4a11652233\
      594708162d111d3c485d23330c581f385148515b7d4d695d5466785d12382533\
      370b211d23165d19627c7a114916013d42735446641d6427471e255916433f2f\
      6520077f45542d02160f367d1d512d21577416300c6c2462264e2c07440f586d\
      0b0e20043c5576400c1c413f300f0d2c375201631c4111561c7c752f45235937\
      505e56763021211c6b2e021f295b08260a157c263d25134a4f32671940072853\
      5c170f1b483e145b5c4a6165303a06567a466e40171f120867776c0c5e442f09\
      5841353d2d363948067a46586f366f052d6b417a513776560d546b5247615523\
      6c652f220711772a124519410b562c7b4b0f5c1c1475277d694f4d3122660167\
      3f0a65127d553548273442682f22237d37276e07620e4522737872175c1d1f7a\
      383f57370c5d6b6e4d2a1501233d5c6b6d0a37530511756045555c504a101144\
      7725385e3d565b0e1a3710296062563c386d411f6619574d4f4c397238420646\
      507f7c1a7108603023522050627b3818213852140e0f310f18464f282e1b4528\
      4a356c122f5b427c6f640f1e2f6e48292e53335a04594c4e2829681835031811\
      6e2f506f7f3748190653233a51254f1469262c1850617033085f4a6a54624529\
      4356705869043c307009254d500614045a1a04767b0d64653f4e3d4714201519\
      51101376762a6468356c5c7e327b611f0815051d0c47720d55600b577c255009\
      204602712d11086a5e661276016b7c0d7b75350b766053537b7a4114593e2039\
      6106701721725e735e0d5c443e2e773b441d650101775e4009542e6425260723\
      47651b3a43696a7a3b1759045b3f055147455675527f5724715f45756f526666\
      62671b205d34397d677615482c775e447335306f504065071b642f4e2008640b\
      18561851591c710349723718181e2b1c2f341a77630231726f2a137721112918\
      537839541d693b154955687c077b1361620b3c16730c636c557b1a7959355e18\
      7a17273013633e3a4f654f7d61573f1c76033f536827227e351d6b5b2f3b2b10\
      75790a702151237a436c15220f7a565552624b372b1b162f775930250d6d370a\
      2c660919317d115d0d306174772c01125e7007492230680703770e605a112b0d\
      07284c352a2d316c167a5c7a0235685d02771c11362d400e413f2f137d702e22\
      6e067c4b742a2b2e44112d3c185c323d263d32174f5c454d1a793b4660526950\
      49474164773c262c3f7e42226a083b320b6f645603204b746c59016a7f031f34\
      42071a2a4a6c334a74150628115d0e297e7f316b79103961324d033e25177b15\
      283c483e4078437b393c2e5b762d235f05704a5c76254f206e31135877412232\
      1a0c663e30581a1a753f4e5e3357752d40081e421120713436021620245e5241\
      7d5d7341121e6f4f276648727613321d765278035c121c2c62393e4948153e45\
      1915251c303941707a47105a7e40607e1a1233620f0f68401e181936303f4d68\
      761c6939525a430b6f3d53444577342f1706317e1f55551a6e3975181b017169\
      1f4f36502607192d437e0929314d116c184a4607562b1606500d331a0a3c5478\
      762e1d7c0a505b1215067556536c207a3f6351682849782907076b061051206d\
      07292016552538655b737213061a44092402060c7637291f066a27253a393647\
      2a3041152f0f18703c4d6727725e65606e324d3a667e1230743e4f41317b7c4d\
      064968433f156379611d3b5b3e406b4846780e3920251a247a0c4a201d1f5c49\
      447f7d662e3f0249726b587713483e09545e31524d3428423f5b020a252d3d01\
      0b57684723155b28163d2543187f466b3f6b1f4d153326536c335e49712e073f\
      045b112e4720575e751d0a06766c6b3630273e721f2b7b1f281c46242a2a2149\
      5712616242675e3238126f0a541816314e1b770814240f6e1c1f226278277567\
      2b043e2d3b72596d07243c155b092d25424e33591b244a086f435a4c022f6a63\
      283a11555a4a2e57095c597032491a5a3348582b5d3514354a11572a71496804\
      773b43234976473867412f63036043184a124f47025e553c550f7f446c7e5711\
      400f3a5f2d7e0a58495c2435735412272f3301753128756637473b611c542053\
      2a3f6051316b617f553321302c2f6d4e0933557d12195c7b1941744a29552912\
      5e521c417a44337436510f1a56783e561b32264e3b7243220410483a095b1504\
      505576331168123b543c18386c083d7202390d1620057b18792177771e264342\
      7539025b686b127b5a5e1b765f2a68402c7e76304c0b4b05151d09275b0b432f\
      63520f281a7d5d170748073a7f04290e183d3213601b3b534e061821780d2d1f\
      07090303687d1c1a4541030d346026632e3e6847427e730b0a224b6e5f74291a\
      49250671393576097b12377566040a47564841525c7c1a35333b4b1576085e29\
      11703f7c766b146e11044c7c7d397b427825653e145c58573017401e27117553\
      4730572b05681d506e5d6a2a6147215d7c1b7b135468577b697e511259561521\
      532e76064f743a387c6538067a76261f4a732c5703663c28463a267773206719\
      1d3242163c56132d22665f18310e2f114a751f14081f32672f1268585f371346\
      0e04666235584f37482607686c56673965687d514d5b5f3f0c4d3f3a0463402d\
      0c6b08732040165e566e567f1417732101166d1739616b38453452274a6c1658\
      31514f6657122b3506776412197e510745186d347b7f5a420f4a2619193f4e53\
      1c68706569254b36101f4c242e4f603e255b266f432e6c264e40593360176b75\
      750e2532435501464a392c2b2572493d6977065e3b2940077b7202243c320322\
      7f6c054941407d4c3919741e5b272d08201008790962261d624d4e30440f1015\
      5d2f2822736e253f277b4a2f7766342d07573d3b2121377a505f2f376962762f\
      5561777374044a0c3948050316245527356d4d0a69777d4d647d471d2a426e7e\
      467970473b62166e3d37490621462706135f564c07701862726f207f39782551\
      693834792a094a586e5e22096e624c2e575c7f7040563041607e63647e4c4230\
      28656c1128400f4f3407391365406b276b2c707d7e6f6c731772385561066d23\
      6a4a3f151f4e0d5d7778352c6e272d773517507b2805757f1346594a6d5033",
);

/// Returns `true` if `path` has a `.frf` extension (case-insensitive).
pub fn is_frf_path(path: &Path) -> bool {
    path.extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("frf"))
}

/// Returns `true` if `path` has an `.odx` extension (case-insensitive).
pub fn is_odx_path(path: &Path) -> bool {
    path.extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("odx"))
}

/// Returns `true` if `data` starts with a VAG `.frf`-encrypted `PK\x03\x04` local ZIP header.
pub fn is_frf_bytes(data: &[u8]) -> bool {
    if data.len() < 30 {
        return false;
    }
    let mut s1 = 0u8;
    let mut s2 = 1u8;
    let mut hdr = [0u8; 4];
    for (i, (&b, &kb)) in data[..4].iter().zip(&FRF_KEY[..4]).enumerate() {
        s1 = s1.wrapping_add(kb).wrapping_mul(3);
        hdr[i] = b ^ s1 ^ 0xFF ^ s2 ^ kb;
        s2 = s2.wrapping_add(1).wrapping_mul(s1);
    }
    hdr == [0x50, 0x4B, 0x03, 0x04]
}

/// Decrypts a VAG `.frf` container into its inner `PK\x03\x04` ZIP archive bytes.
pub fn decrypt_frf_to_zip(frf: &[u8]) -> Result<Vec<u8>, String> {
    if frf.len() < 30 {
        return Err(format!("FRF file too small ({} bytes)", frf.len()));
    }
    if frf.len() > MAX_FRF_BYTES {
        return Err(format!(
            "FRF file exceeds {} MiB safety limit ({} bytes)",
            MAX_FRF_BYTES / (1024 * 1024),
            frf.len()
        ));
    }
    let mut out = Vec::with_capacity(frf.len());
    let mut first_seed = 0u8;
    let mut second_seed = 1u8;
    let mut ki = 0usize;
    for &b in frf {
        let kb = FRF_KEY[ki];
        first_seed = first_seed.wrapping_add(kb).wrapping_mul(3);
        out.push(b ^ first_seed ^ 0xFF ^ second_seed ^ kb);
        second_seed = second_seed.wrapping_add(1).wrapping_mul(first_seed);
        ki += 1;
        if ki == FRF_KEY.len() {
            ki = 0;
        }
    }
    if out.get(..4) != Some(&[0x50, 0x4B, 0x03, 0x04]) {
        return Err("FRF decryption did not produce a valid PKZIP header".into());
    }
    Ok(out)
}

fn u16_le(buf: &[u8], off: usize) -> Result<u16, String> {
    let s = buf
        .get(off..off.checked_add(2).ok_or("ZIP offset overflow")?)
        .ok_or("ZIP header truncated")?;
    Ok(u16::from_le_bytes([s[0], s[1]]))
}

fn u32_le(buf: &[u8], off: usize) -> Result<u32, String> {
    let s = buf
        .get(off..off.checked_add(4).ok_or("ZIP offset overflow")?)
        .ok_or("ZIP header truncated")?;
    Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}

/// Extracts `(entry_name, odx_xml_string)` from a decrypted FRF ZIP archive, verifying IEEE CRC-32
/// and enforcing `MAX_ODX_BYTES` decompression bounds.
pub fn extract_odx_from_zip(zip: &[u8]) -> Result<(String, String), String> {
    if zip.len() < 22 {
        return Err("ZIP archive too small".into());
    }
    let min_eocd = zip.len().saturating_sub(22 + 65_535);
    let mut eocd_pos = None;
    for pos in (min_eocd..=zip.len() - 22).rev() {
        if zip[pos..pos + 4] == [0x50, 0x4B, 0x05, 0x06] {
            let comment_len = u16_le(zip, pos + 20)? as usize;
            if pos + 22 + comment_len == zip.len() {
                eocd_pos = Some(pos);
                break;
            }
        }
    }
    let eocd = eocd_pos.ok_or("ZIP End-of-Central-Directory record not found")?;
    let cd_entries = u16_le(zip, eocd + 10)? as usize;
    let cd_size = u32_le(zip, eocd + 12)? as usize;
    let cd_offset = u32_le(zip, eocd + 16)? as usize;
    let cd_end = cd_offset
        .checked_add(cd_size)
        .filter(|&end| end <= eocd)
        .ok_or("ZIP Central Directory out of bounds")?;
    if cd_entries == 0 {
        return Err("ZIP archive contains no entries".into());
    }

    let mut pos = cd_offset;
    let mut chosen: Option<(String, u16, u32, usize, usize, usize)> = None;
    for _ in 0..cd_entries {
        if pos.checked_add(46).is_none_or(|end| end > cd_end) {
            return Err("ZIP Central Directory entry truncated".into());
        }
        if zip[pos..pos + 4] != [0x50, 0x4B, 0x01, 0x02] {
            return Err("invalid ZIP Central Directory file header signature".into());
        }
        let flags = u16_le(zip, pos + 8)?;
        if flags & 0x0001 != 0 {
            return Err("encrypted ZIP entries inside FRF are not supported".into());
        }
        let method = u16_le(zip, pos + 10)?;
        let crc32_expected = u32_le(zip, pos + 16)?;
        let comp_size = u32_le(zip, pos + 20)? as usize;
        let uncomp_size = u32_le(zip, pos + 24)? as usize;
        let name_len = u16_le(zip, pos + 28)? as usize;
        let extra_len = u16_le(zip, pos + 30)? as usize;
        let comment_len = u16_le(zip, pos + 32)? as usize;
        let lh_offset = u32_le(zip, pos + 42)? as usize;

        let next_pos = pos
            .checked_add(46)
            .and_then(|p| p.checked_add(name_len))
            .and_then(|p| p.checked_add(extra_len))
            .and_then(|p| p.checked_add(comment_len))
            .filter(|&p| p <= cd_end)
            .ok_or("ZIP Central Directory variable fields out of bounds")?;

        let name_bytes = &zip[pos + 46..pos + 46 + name_len];
        let entry_name = String::from_utf8_lossy(name_bytes).into_owned();
        let is_dir = entry_name.ends_with('/') || entry_name.ends_with('\\');
        if !is_dir {
            let is_odx = Path::new(&entry_name)
                .extension()
                .and_then(|e| e.to_str())
                .is_some_and(|e| e.eq_ignore_ascii_case("odx"));
            if is_odx || chosen.is_none() {
                chosen = Some((
                    entry_name,
                    method,
                    crc32_expected,
                    comp_size,
                    uncomp_size,
                    lh_offset,
                ));
                if is_odx {
                    break;
                }
            }
        }
        pos = next_pos;
    }

    let (entry_name, method, crc32_expected, comp_size, uncomp_size, lh_offset) =
        chosen.ok_or("no ODX file entry found in FRF ZIP archive")?;
    if uncomp_size > MAX_ODX_BYTES {
        return Err(format!(
            "ODX entry {entry_name:?} exceeds {} MiB limit ({uncomp_size} bytes)",
            MAX_ODX_BYTES / (1024 * 1024)
        ));
    }
    if lh_offset.checked_add(30).is_none_or(|end| end > cd_offset) {
        return Err("ZIP Local File Header out of bounds".into());
    }
    if zip[lh_offset..lh_offset + 4] != [0x50, 0x4B, 0x03, 0x04] {
        return Err("invalid ZIP Local File Header signature".into());
    }
    let lh_name_len = u16_le(zip, lh_offset + 26)? as usize;
    let lh_extra_len = u16_le(zip, lh_offset + 28)? as usize;
    let data_start = lh_offset
        .checked_add(30)
        .and_then(|p| p.checked_add(lh_name_len))
        .and_then(|p| p.checked_add(lh_extra_len))
        .ok_or("ZIP Local File Header overflow")?;
    let data_end = data_start
        .checked_add(comp_size)
        .filter(|&end| end <= cd_offset)
        .ok_or("ZIP compressed payload out of bounds")?;
    let payload = &zip[data_start..data_end];

    let odx_bytes = match method {
        0 => payload.to_vec(),
        8 => miniz_oxide::inflate::decompress_to_vec_with_limit(payload, MAX_ODX_BYTES)
            .map_err(|e| format!("DEFLATE decompression failed for {entry_name:?}: {e:?}"))?,
        other => return Err(format!("unsupported ZIP compression method {other} for {entry_name:?}")),
    };
    if odx_bytes.len() != uncomp_size {
        return Err(format!(
            "ODX entry {entry_name:?} uncompressed size mismatch (got {}, expected {uncomp_size})",
            odx_bytes.len()
        ));
    }
    let crc32_actual = dq250_crc_register(&odx_bytes) ^ 0xFFFF_FFFF;
    if crc32_actual != crc32_expected {
        return Err(format!(
            "ODX entry {entry_name:?} CRC-32 mismatch (computed 0x{crc32_actual:08X}, expected 0x{crc32_expected:08X})"
        ));
    }
    let odx_str = String::from_utf8(odx_bytes)
        .unwrap_or_else(|e| String::from_utf8_lossy(&e.into_bytes()).into_owned());
    Ok((entry_name, odx_str))
}

// =================================================================================================
// Pure-Rust FIPS-197 AES-128-CBC Decryption
// =================================================================================================

const AES_SBOX: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
];

const AES_INV_SBOX: [u8; 256] = {
    let mut inv = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        inv[AES_SBOX[i] as usize] = i as u8;
        i += 1;
    }
    inv
};

const fn gf_mul(mut a: u8, mut b: u8) -> u8 {
    let mut p = 0u8;
    let mut i = 0;
    while i < 8 {
        if b & 1 != 0 {
            p ^= a;
        }
        let hi = a & 0x80;
        a <<= 1;
        if hi != 0 {
            a ^= 0x1b;
        }
        b >>= 1;
        i += 1;
    }
    p
}

const fn gf_table(factor: u8) -> [u8; 256] {
    let mut t = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        t[i] = gf_mul(i as u8, factor);
        i += 1;
    }
    t
}

const GF_MUL9: [u8; 256] = gf_table(9);
const GF_MUL11: [u8; 256] = gf_table(11);
const GF_MUL13: [u8; 256] = gf_table(13);
const GF_MUL14: [u8; 256] = gf_table(14);

fn expand_key_128(key: &[u8; 16]) -> [[u8; 16]; 11] {
    const RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];
    let mut rk = [[0u8; 16]; 11];
    rk[0] = *key;
    for r in 0..10 {
        let prev = rk[r];
        let mut next = [0u8; 16];
        next[0] = prev[0] ^ AES_SBOX[prev[13] as usize] ^ RCON[r];
        next[1] = prev[1] ^ AES_SBOX[prev[14] as usize];
        next[2] = prev[2] ^ AES_SBOX[prev[15] as usize];
        next[3] = prev[3] ^ AES_SBOX[prev[12] as usize];
        for i in 4..16 {
            next[i] = prev[i] ^ next[i - 4];
        }
        rk[r + 1] = next;
    }
    rk
}

#[inline]
fn aes128_decrypt_block(rk: &[[u8; 16]; 11], block: &[u8; 16]) -> [u8; 16] {
    let mut s = *block;
    for (b, &k) in s.iter_mut().zip(&rk[10]) {
        *b ^= k;
    }
    for round in (1..=9).rev() {
        s = [
            AES_INV_SBOX[s[0] as usize],
            AES_INV_SBOX[s[13] as usize],
            AES_INV_SBOX[s[10] as usize],
            AES_INV_SBOX[s[7] as usize],
            AES_INV_SBOX[s[4] as usize],
            AES_INV_SBOX[s[1] as usize],
            AES_INV_SBOX[s[14] as usize],
            AES_INV_SBOX[s[11] as usize],
            AES_INV_SBOX[s[8] as usize],
            AES_INV_SBOX[s[5] as usize],
            AES_INV_SBOX[s[2] as usize],
            AES_INV_SBOX[s[15] as usize],
            AES_INV_SBOX[s[12] as usize],
            AES_INV_SBOX[s[9] as usize],
            AES_INV_SBOX[s[6] as usize],
            AES_INV_SBOX[s[3] as usize],
        ];
        for (b, &k) in s.iter_mut().zip(&rk[round]) {
            *b ^= k;
        }
        for c in 0..4 {
            let i = 4 * c;
            let (s0, s1, s2, s3) = (s[i] as usize, s[i + 1] as usize, s[i + 2] as usize, s[i + 3] as usize);
            s[i] = GF_MUL14[s0] ^ GF_MUL11[s1] ^ GF_MUL13[s2] ^ GF_MUL9[s3];
            s[i + 1] = GF_MUL9[s0] ^ GF_MUL14[s1] ^ GF_MUL11[s2] ^ GF_MUL13[s3];
            s[i + 2] = GF_MUL13[s0] ^ GF_MUL9[s1] ^ GF_MUL14[s2] ^ GF_MUL11[s3];
            s[i + 3] = GF_MUL11[s0] ^ GF_MUL13[s1] ^ GF_MUL9[s2] ^ GF_MUL14[s3];
        }
    }
    [
        AES_INV_SBOX[s[0] as usize] ^ rk[0][0],
        AES_INV_SBOX[s[13] as usize] ^ rk[0][1],
        AES_INV_SBOX[s[10] as usize] ^ rk[0][2],
        AES_INV_SBOX[s[7] as usize] ^ rk[0][3],
        AES_INV_SBOX[s[4] as usize] ^ rk[0][4],
        AES_INV_SBOX[s[1] as usize] ^ rk[0][5],
        AES_INV_SBOX[s[14] as usize] ^ rk[0][6],
        AES_INV_SBOX[s[11] as usize] ^ rk[0][7],
        AES_INV_SBOX[s[8] as usize] ^ rk[0][8],
        AES_INV_SBOX[s[5] as usize] ^ rk[0][9],
        AES_INV_SBOX[s[2] as usize] ^ rk[0][10],
        AES_INV_SBOX[s[15] as usize] ^ rk[0][11],
        AES_INV_SBOX[s[12] as usize] ^ rk[0][12],
        AES_INV_SBOX[s[9] as usize] ^ rk[0][13],
        AES_INV_SBOX[s[6] as usize] ^ rk[0][14],
        AES_INV_SBOX[s[3] as usize] ^ rk[0][15],
    ]
}

/// Decrypts AES-128-CBC `ciphertext` without PKCS#7 unpadding (SIMOS18 CBOOT zero-pads to 16 bytes
/// and terminates decompression at `<UNCOMPRESSED-SIZE>`).
pub fn aes128_cbc_decrypt(key: &[u8; 16], iv: &[u8; 16], ciphertext: &[u8]) -> Result<Vec<u8>, String> {
    if ciphertext.is_empty() || !ciphertext.len().is_multiple_of(16) {
        return Err(format!(
            "AES-128-CBC input length ({}) must be a non-zero multiple of 16",
            ciphertext.len()
        ));
    }
    let rk = expand_key_128(key);
    let mut out = Vec::with_capacity(ciphertext.len());
    let mut prev = *iv;
    for chunk in ciphertext.as_chunks::<16>().0 {
        let mut plain = aes128_decrypt_block(&rk, chunk);
        for (p, &v) in plain.iter_mut().zip(&prev) {
            *p ^= v;
        }
        out.extend_from_slice(&plain);
        prev = *chunk;
    }
    Ok(out)
}

// =================================================================================================
// Clean-Room Continental SIMOS18.1 CBOOT LZSS Decompressor (CBOOT 0x8002EC94..0x8002F0B6)
// =================================================================================================

/// Clean-room implementation of the Continental SIMOS18.1 CBOOT LZSS decompressor (`0x8002EC94..0x8002F0B6`,
/// REA `ev_aff0db1293c9a949cf3b04d261f112411a5aaa3ba08a5b8ee34d074db05a9875`).
pub fn cboot_lzss_decompress(src: &[u8], expected_len: usize) -> Result<Vec<u8>, String> {
    if expected_len > SIMOS18_FLASH_LEN {
        return Err(format!("implausible LZSS target length {expected_len}"));
    }
    let mut win = [0u8; CBOOT_LZSS_WIN_LEN];
    let mut win_pos = 0usize;
    let mut out = Vec::with_capacity(expected_len);
    let mut in_pos = 0usize;

    while in_pos < src.len() && out.len() < expected_len {
        let flags = src[in_pos];
        in_pos += 1;
        for bit in (0..8).rev() {
            if in_pos >= src.len() || out.len() >= expected_len {
                break;
            }
            if ((flags >> bit) & 1) == 0 {
                let b = src[in_pos];
                in_pos += 1;
                win[win_pos] = b;
                win_pos += 1;
                if win_pos >= CBOOT_LZSS_WIN_LEN {
                    win_pos = 0;
                }
                out.push(b);
            } else {
                if in_pos + 1 >= src.len() {
                    in_pos = src.len();
                    break;
                }
                let b0 = src[in_pos] as usize;
                let b1 = src[in_pos + 1] as usize;
                in_pos += 2;
                let length = b0 >> 2;
                let offset = ((b0 & 0x03) << 8) | b1;
                if offset < length || offset > CBOOT_LZSS_WIN_LEN {
                    return Err(format!(
                        "corrupt CBOOT LZSS back-reference (len={length}, offset={offset}) at byte {in_pos}"
                    ));
                }
                for _ in 0..length {
                    if out.len() >= expected_len {
                        break;
                    }
                    let src_idx = if win_pos >= offset {
                        win_pos - offset
                    } else {
                        win_pos + CBOOT_LZSS_WIN_LEN - offset
                    };
                    let b = win[src_idx];
                    win[win_pos] = b;
                    win_pos += 1;
                    if win_pos >= CBOOT_LZSS_WIN_LEN {
                        win_pos = 0;
                    }
                    out.push(b);
                }
            }
        }
    }

    if out.len() != expected_len {
        return Err(format!(
            "truncated CBOOT LZSS stream: produced {} bytes, expected {expected_len}",
            out.len()
        ));
    }
    Ok(out)
}

// =================================================================================================
// ODX XML Flashdata Extraction & Image Assembly
// =================================================================================================

fn extract_tag_text<'a>(xml: &'a str, tag: &str) -> Option<&'a str> {
    let open_pat = format!("<{tag}");
    let close_pat = format!("</{tag}>");
    let mut search_from = 0usize;
    while let Some(rel) = xml[search_from..].find(&open_pat) {
        let start_tag = search_from + rel;
        let after_name = start_tag + open_pat.len();
        let next_b = *xml.as_bytes().get(after_name)?;
        if next_b != b'>' && next_b != b'/' && !next_b.is_ascii_whitespace() {
            search_from = after_name;
            continue;
        }
        let close_open = xml[after_name..].find('>')? + after_name;
        if xml.as_bytes().get(close_open.wrapping_sub(1)) == Some(&b'/') {
            return Some("");
        }
        let content_start = close_open + 1;
        let end_rel = xml[content_start..].find(&close_pat)?;
        return Some(&xml[content_start..content_start + end_rel]);
    }
    None
}

fn extract_attr<'a>(open_tag: &'a str, attr: &str) -> Option<&'a str> {
    let pat = format!("{attr}=\"");
    let s = open_tag.find(&pat)? + pat.len();
    let e = open_tag[s..].find('"')? + s;
    Some(&open_tag[s..e])
}

/// Parses `<DATABLOCK ... TYPE="DATA">` mappings: `flashdata_id -> (block_num, uncompressed_size)`.
fn parse_odx_datablocks(odx: &str) -> HashMap<String, (u8, usize)> {
    let mut map = HashMap::new();
    let mut rest = odx;
    while let Some(s) = rest.find("<DATABLOCK ") {
        rest = &rest[s..];
        let Some(end_open) = rest.find('>') else { break };
        let open_tag = &rest[..end_open + 1];
        let Some(end_blk) = rest.find("</DATABLOCK>") else { break };
        let body = &rest[end_open + 1..end_blk];
        rest = &rest[end_blk + "</DATABLOCK>".len()..];

        if extract_attr(open_tag, "TYPE").is_some_and(|t| t.eq_ignore_ascii_case("ERASE")) {
            continue;
        }
        let Some(fd_pos) = body.find("<FLASHDATA-REF") else { continue };
        let Some(fd_end) = body[fd_pos..].find('>') else { continue };
        let Some(fd_id) = extract_attr(&body[fd_pos..fd_pos + fd_end + 1], "ID-REF") else {
            continue;
        };
        let block_num = extract_tag_text(body, "SOURCE-START-ADDRESS")
            .and_then(|t| u8::from_str_radix(t.trim(), 16).ok());
        let uncomp_len = extract_tag_text(body, "UNCOMPRESSED-SIZE")
            .and_then(|t| t.trim().parse::<usize>().ok());
        if let (Some(bn), Some(ul)) = (block_num, uncomp_len) {
            map.insert(fd_id.to_string(), (bn, ul));
        }
    }
    map
}

fn decode_hex_whitespace_tolerant(s: &str, max_bytes: Option<usize>) -> Result<Vec<u8>, String> {
    let cap = max_bytes.unwrap_or(s.len() / 2);
    let mut out = Vec::with_capacity(cap.min(1024 * 1024));
    let mut hi: Option<u8> = None;
    for b in s.bytes() {
        if b.is_ascii_whitespace() {
            continue;
        }
        let nib = match b {
            b'0'..=b'9' => b - b'0',
            b'a'..=b'f' => b - b'a' + 10,
            b'A'..=b'F' => b - b'A' + 10,
            _ => return Err(format!("invalid hex character {:?} in ODX <DATA>", b as char)),
        };
        if let Some(h) = hi.take() {
            out.push((h << 4) | nib);
            if max_bytes.is_some_and(|m| out.len() >= m) {
                return Ok(out);
            }
        } else {
            hi = Some(nib);
        }
    }
    if hi.is_some() {
        return Err("odd number of hex digits in ODX <DATA>".into());
    }
    Ok(out)
}

fn lookup_simos18_block(block_num: u8) -> Option<(&'static str, usize, usize)> {
    SIMOS18_FRF_BLOCKS
        .iter()
        .find(|&&(n, _, _, _)| n == block_num)
        .map(|&(_, name, off, sz)| (name, off, sz))
}

fn infer_block_num_from_fd_id(fd_id: &str) -> Option<u8> {
    if fd_id.to_ascii_uppercase().contains("ERASE") {
        return None;
    }
    let idx = fd_id.rsplit("FD_").next()?.trim().parse::<u8>().ok()?;
    idx.checked_add(1)
}

fn decode_odx_block(method: &str, raw_bytes: &[u8], expected_len: usize) -> Result<Vec<u8>, String> {
    let m = method.trim().as_bytes();
    if m.len() != 2 {
        return Err(format!("unsupported ODX ENCRYPT-COMPRESS-METHOD {method:?}"));
    }
    let decrypted: std::borrow::Cow<'_, [u8]> = match m[0].to_ascii_uppercase() {
        b'0' => std::borrow::Cow::Borrowed(raw_bytes),
        b'A' => std::borrow::Cow::Owned(aes128_cbc_decrypt(&S18_AES_KEY, &S18_AES_IV, raw_bytes)?),
        other => {
            return Err(format!(
                "unsupported ODX encryption method {:?} in {method:?}",
                other as char
            ))
        }
    };
    match m[1].to_ascii_uppercase() {
        b'0' => {
            if decrypted.len() < expected_len {
                return Err(format!(
                    "uncompressed ODX block shorter than expected ({} < {expected_len})",
                    decrypted.len()
                ));
            }
            Ok(decrypted[..expected_len].to_vec())
        }
        b'A' => cboot_lzss_decompress(&decrypted, expected_len),
        other => Err(format!(
            "unsupported ODX compression method {:?} in {method:?}",
            other as char
        )),
    }
}

fn extract_cal_metadata(cal_slice: &[u8]) -> (String, String) {
    let box_code = cal_slice
        .get(2..8)
        .map(|b| String::from_utf8_lossy(b).trim().to_string())
        .unwrap_or_default();
    let head = cal_slice.get(..0xB0).unwrap_or(cal_slice);
    let part = head
        .split(|b| !b.is_ascii_alphanumeric())
        .filter_map(|t| std::str::from_utf8(t).ok())
        .find(|t| (9..=11).contains(&t.len()) && (t.get(3..6) == Some("906") || t.get(3..6) == Some("907")))
        .unwrap_or_default()
        .to_string();
    (box_code, part)
}

/// Fast header peek: decrypts and decompresses ONLY the first `prefix_len` bytes of the `CAL`
/// (`FD_4`) block inside `odx_xml`, avoiding full 3 MB ASW decompression when scanning image lists.
pub fn peek_odx_cal_prefix(odx_xml: &str, prefix_len: usize) -> Result<Vec<u8>, String> {
    if prefix_len == 0 || prefix_len > 0x07_FC00 {
        return Err(format!("invalid CAL prefix length {prefix_len}"));
    }
    let db_map = parse_odx_datablocks(odx_xml);
    let mut rest = odx_xml;
    while let Some(s) = rest.find("<FLASHDATA ") {
        rest = &rest[s..];
        let Some(end_open) = rest.find('>') else { break };
        let open_tag = &rest[..end_open + 1];
        let Some(end_fd) = rest.find("</FLASHDATA>") else { break };
        let body = &rest[end_open + 1..end_fd];
        rest = &rest[end_fd + "</FLASHDATA>".len()..];

        let Some(fd_id) = extract_attr(open_tag, "ID") else { continue };
        let block_num = db_map
            .get(fd_id)
            .map(|&(bn, _)| bn)
            .or_else(|| infer_block_num_from_fd_id(fd_id));
        if block_num != Some(5) {
            continue;
        }
        let method = extract_tag_text(body, "ENCRYPT-COMPRESS-METHOD").unwrap_or("00");
        let data_hex = extract_tag_text(body, "DATA").ok_or("missing <DATA> in CAL FLASHDATA")?;
        // Each 2-byte LZSS back-ref emits >= 1 byte (or 1 literal byte per 1 input byte plus 1 flag byte per 8 tokens),
        // so `prefix_len * 3 + 64` aligned up to 16 bytes strictly bounds the needed AES-CBC prefix.
        let max_ct = (prefix_len.saturating_mul(3).saturating_add(64) + 15) & !15;
        let raw = decode_hex_whitespace_tolerant(data_hex, Some(max_ct))?;
        let aligned_len = if method.trim().as_bytes().first().is_some_and(|c| c.eq_ignore_ascii_case(&b'A')) {
            raw.len() & !15
        } else {
            raw.len()
        };
        return decode_odx_block(method, &raw[..aligned_len], prefix_len);
    }
    Err("CAL block (FD_4 / block 5) not found in ODX".into())
}

/// Fast header peek on a VAG `.frf` container: extracts only the first `prefix_len` bytes of `CAL` (`0x200000..`).
pub fn peek_frf_cal_prefix(frf_bytes: &[u8], prefix_len: usize) -> Result<Vec<u8>, String> {
    let zip = decrypt_frf_to_zip(frf_bytes)?;
    let (_, odx_xml) = extract_odx_from_zip(&zip)?;
    peek_odx_cal_prefix(&odx_xml, prefix_len)
}

/// Unpacks an ODX XML string into a 4 MB SIMOS18.1 PFlash image.
///
/// `sboot_or_donor` accepts:
/// - `Some(donor)` of `0x40_0000` (4 MiB) or `0x30_0000` (3 MiB BSL read, zero-padded to 4 MiB): starts from `donor`,
///   preserving its `SBOOT` (`0x00000..0x1C000`), chip UID (`0x14200`), and outside-block bytes.
/// - `Some(sboot)` of `0x01_C000` (112 KiB standalone `sboot.bin`): starts from TC1791 erased PFlash (`0x00`), places
///   `sboot` at `0x00000..0x1C000`, unpacks all blocks, and writes CBOOT post-programming markers.
/// - `None`: starts from erased PFlash (`0x00`), unpacks all blocks, and writes CBOOT post-programming markers.
///
/// Verifies every present block CRC-32 and recognised ECM3 sum via [`verify_checksums`].
pub fn unpack_odx_into_image(
    odx_xml: &str,
    odx_name: &str,
    sboot_or_donor: Option<&[u8]>,
) -> Result<(Vec<u8>, FrfUnpackReport), String> {
    let mut needs_markers = false;
    let mut img = match sboot_or_donor {
        Some(d) if d.len() == SIMOS18_FLASH_LEN => d.to_vec(),
        Some(d) if d.len() == 0x30_0000 => {
            let mut v = vec![0u8; SIMOS18_FLASH_LEN];
            v[..0x30_0000].copy_from_slice(d);
            v
        }
        Some(sboot) if sboot.len() == SIMOS18_SBOOT_LEN => {
            let mut v = vec![0u8; SIMOS18_FLASH_LEN];
            v[..SIMOS18_SBOOT_LEN].copy_from_slice(sboot);
            needs_markers = true;
            v
        }
        Some(other) => {
            return Err(format!(
                "SBOOT / donor image must be 112 KB (0x1C000), 3 MB (0x300000), or 4 MB (0x400000); got {} bytes",
                other.len()
            ));
        }
        None => {
            needs_markers = true;
            vec![0u8; SIMOS18_FLASH_LEN]
        }
    };

    let db_map = parse_odx_datablocks(odx_xml);
    let mut blocks_written = Vec::new();
    let mut rest = odx_xml;
    while let Some(s) = rest.find("<FLASHDATA ") {
        rest = &rest[s..];
        let Some(end_open) = rest.find('>') else { break };
        let open_tag = &rest[..end_open + 1];
        let Some(end_fd) = rest.find("</FLASHDATA>") else { break };
        let body = &rest[end_open + 1..end_fd];
        rest = &rest[end_fd + "</FLASHDATA>".len()..];

        let Some(fd_id) = extract_attr(open_tag, "ID") else { continue };
        if fd_id.to_ascii_uppercase().contains("ERASE") {
            continue;
        }
        let Some(data_hex) = extract_tag_text(body, "DATA") else { continue };
        if data_hex.trim().is_empty() {
            continue;
        }
        let method = extract_tag_text(body, "ENCRYPT-COMPRESS-METHOD").unwrap_or("00");
        let (block_num, odx_uncomp_len) = match db_map.get(fd_id) {
            Some(&(bn, ul)) => (bn, Some(ul)),
            None => (
                infer_block_num_from_fd_id(fd_id)
                    .ok_or_else(|| format!("unrecognised FLASHDATA ID {fd_id:?} in {odx_name}"))?,
                None,
            ),
        };
        let (block_name, file_off, max_slot_len) = lookup_simos18_block(block_num)
            .ok_or_else(|| format!("unsupported SIMOS18 logical block #{block_num} ({fd_id}) in {odx_name}"))?;
        let target_len = odx_uncomp_len.unwrap_or(max_slot_len);
        if target_len == 0 || target_len > max_slot_len || file_off + target_len > SIMOS18_FLASH_LEN {
            return Err(format!(
                "{odx_name}: block {block_name} uncompressed size 0x{target_len:X} exceeds slot 0x{max_slot_len:X}"
            ));
        }

        let raw_bytes = decode_hex_whitespace_tolerant(data_hex, None)?;
        let unpacked = decode_odx_block(method, &raw_bytes, target_len)
            .map_err(|e| format!("{odx_name} ({block_name}): {e}"))?;
        img[file_off..file_off + target_len].copy_from_slice(&unpacked);
        blocks_written.push(block_name);
    }

    if blocks_written.is_empty() {
        return Err(format!("{odx_name}: no programmable SIMOS18 FLASHDATA blocks found"));
    }

    if needs_markers {
        // CBOOT writes a block-start marker at `+FC00` before programming ASW1..3 / CAL, and block-valid
        // markers at `+FE00` after programming (`apply_post_programming`).
        for &(blk_name, at, marker) in &[
            ("ASW1", 0x13_FC00usize, 0x9048_2490u32),
            ("ASW2", 0x1F_FC00, 0x9048_2490),
            ("ASW3", 0x2F_FC00, 0x9048_2490),
            ("CAL", 0x27_FC00, 0x4290_4824),
        ] {
            if blocks_written.contains(&blk_name) {
                img[at..at + 4].copy_from_slice(&marker.to_le_bytes());
            }
        }
        let _ = apply_post_programming(&mut img)?;
    }

    let cks = verify_checksums(&img)?;
    if !cks.ok() {
        let failed: Vec<String> = cks
            .entries
            .iter()
            .filter(|e| !e.ok())
            .map(|e| format!("{} (stored 0x{:X} != computed 0x{:X})", e.name, e.stored, e.computed))
            .collect();
        return Err(format!(
            "{odx_name}: unpacked image checksum verification failed: {}",
            failed.join(", ")
        ));
    }

    let (box_code, part_number) = extract_cal_metadata(&img[0x20_0000..0x28_0000]);
    let has_sboot = img[..0x1000].iter().any(|&b| b != 0x00 && b != 0xFF);
    Ok((
        img,
        FrfUnpackReport {
            odx_name: odx_name.to_string(),
            blocks_written,
            box_code,
            part_number,
            has_sboot,
        },
    ))
}

/// Unpacks a VAG ODIS `.frf` container directly into a 4 MB SIMOS18.1 PFlash image in memory.
pub fn unpack_frf_into_image(
    frf_bytes: &[u8],
    donor_4mb: Option<&[u8]>,
) -> Result<(Vec<u8>, FrfUnpackReport), String> {
    let zip = decrypt_frf_to_zip(frf_bytes)?;
    let (odx_name, odx_xml) = extract_odx_from_zip(&zip)?;
    unpack_odx_into_image(&odx_xml, &odx_name, donor_4mb)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aes128_decrypt_matches_nist_fips197_appendix_b_vector() {
        let key = decode_hex::<16>(b"000102030405060708090a0b0c0d0e0f");
        let ct = decode_hex::<16>(b"69c4e0d86a7b0430d8cdb78070b4c55a");
        let expected_pt = decode_hex::<16>(b"00112233445566778899aabbccddeeff");
        let rk = expand_key_128(&key);
        assert_eq!(aes128_decrypt_block(&rk, &ct), expected_pt);

        let iv = [0xA5u8; 16];
        let mut expected_cbc = expected_pt;
        for (p, &v) in expected_cbc.iter_mut().zip(&iv) {
            *p ^= v;
        }
        assert_eq!(aes128_cbc_decrypt(&key, &iv, &ct).unwrap(), expected_cbc);
    }

    #[test]
    fn cboot_lzss_handles_literals_backrefs_zero_window_and_rejects_bad_offsets() {
        // Flag 0x02 (0b0000_0010): 6 literals, 1 back-ref (len=4, off=4), 1 literal;
        // Flag 0x40 (0b0100_0000): 1 literal, 1 back-ref into zero-init window (len=8, off=0x200).
        let stream = [
            0x02, 0xE8, 0xCA, 0x01, 0x80, 0x42, 0xE0, 0x10, 0x04, 0x01, // 11 bytes emitted
            0x40, 0x80, 0x22, 0x00, // +1 literal (0x80) + 8 zeros from initial window (off=0x200) = 20 bytes
        ];
        let out = cboot_lzss_decompress(&stream, 20).unwrap();
        assert_eq!(
            out,
            vec![
                0xE8, 0xCA, 0x01, 0x80, 0x42, 0xE0, 0x01, 0x80, 0x42, 0xE0, 0x01, 0x80, 0, 0, 0, 0,
                0, 0, 0, 0
            ]
        );
        // Truncated stream (asks for 25 bytes when only 20 exist) must error cleanly
        assert!(cboot_lzss_decompress(&stream, 25).is_err());
        // Invalid back-reference (`offset < length`: len=5, off=2 -> b0 = 5<<2 = 0x14, b1 = 0x02) must error
        let bad = [0x80, 0x14, 0x02];
        assert!(cboot_lzss_decompress(&bad, 5).is_err());
    }

    #[test]
    fn rejects_malformed_or_truncated_frf_and_zip_inputs_without_panic() {
        assert!(!is_frf_bytes(&[]));
        assert!(!is_frf_bytes(&[0u8; 64]));
        assert!(decrypt_frf_to_zip(&[0u8; 10]).is_err());
        assert!(decrypt_frf_to_zip(&[0xAAu8; 64]).is_err());
        assert!(extract_odx_from_zip(b"PK\x03\x04not_a_real_zip_archive_at_all").is_err());
    }
}
