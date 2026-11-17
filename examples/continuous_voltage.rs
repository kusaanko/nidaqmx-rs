use nidaqmx_rs::{get_system_devices, SampleMode, Task};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let devices = get_system_devices()?;
    println!("Devices: {devices:?}");
    let Some(device) = devices.first() else {
        return Ok(());
    };
    // Construct the task inside the acquisition thread.
    let task = Task::new("")?;
    for channel in 0..3 {
        task.ai_channels
            .add_ai_voltage_chan(&format!("{device}/ai{channel}"), -10.0, 10.0)?;
    }
    let rate: f64 = 100_000.0;
    task.timing
        .cfg_samp_clk_timing(rate, SampleMode::ContinuousSamps, rate.ceil() as u64)?;
    task.start()?;
    for _ in 0..10 {
        let data = task.read_sync(10_000)?;
        let count = data.len() / 3;
        if count > 0 {
            let channels: Vec<_> = data.chunks_exact(count).collect();
            println!(
                "LC / Acc / Mic: {count} samples each, first values: {:?}",
                channels
                    .iter()
                    .map(|channel| channel[0])
                    .collect::<Vec<_>>()
            );
        }
    }
    // Dropping the task stops acquisition and releases driver resources.
    Ok(())
}
