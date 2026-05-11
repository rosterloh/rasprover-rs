use core::fmt::Write;
use defmt::info;
use heapless::String;

pub const fn or_str(opt: Option<&'static str>, default: &'static str) -> &'static str {
    if let Some(val) = opt {
        val
    } else if opt.is_none() {
        default
    } else {
        unreachable!()
    }
}

pub fn log_banner(msg: &str) {
    const WIDTH: usize = 50;
    let mut s: String<128> = String::new();
    let msg_len = msg.len() + 2;
    let stars = (WIDTH.saturating_sub(msg_len)) / 2;

    for _ in 0..stars {
        s.push('*').ok();
    }
    write!(s, " {} ", msg).ok();
    while s.len() < WIDTH {
        s.push('*').ok();
    }

    info!("{}", s);
}
