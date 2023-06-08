mod control_main;
pub mod plottable;

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions::default();

    eframe::run_native(
        "Plotter",
        native_options,
        Box::new(|cc| Box::new(plotter::PlotterApp::start(cc, control_main::init_control))),
    )
}
