fn main() {
    // Embed the application icon and version info in the Windows executable.
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        let mut res = winresource::WindowsResource::new();
        res.set_icon("assets/actvib.ico");
        res.set("ProductName", "ActVib");
        res.set("FileDescription", "ActVib - active vibration control");
        if let Err(e) = res.compile() {
            println!("cargo:warning=could not embed Windows resources: {e}");
        }
    }
}
