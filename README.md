# nidaqmx-rs

Synchronous NI-DAQmx device enumeration and multi-channel analog voltage input.
The raw C bindings are generated using bindgen from the installed header at
`C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\include\NIDAQmx.h`.
The current build links the installed Windows x64 import library in
`C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\lib64\msvc`; an installed
NI-DAQmx runtime is required to run applications and tests. Building requires
Rust, MSVC build tools, and libclang for bindgen.

```rust,no_run
use nidaqmx_rs::{get_system_devices, SampleMode, Task};

fn acquire() -> Result<(), nidaqmx_rs::Error> {
    let devices = get_system_devices()?;
    let Some(device) = devices.first() else { return Ok(()); };
    let task = Task::new("")?;
    for channel in 0..3 {
        task.ai_channels.add_ai_voltage_chan(
            &format!("{device}/ai{channel}"), -10.0, 10.0)?;
    }
    let rate: f64 = 100_000.0;
    task.timing.cfg_samp_clk_timing(
        rate, SampleMode::ContinuousSamps, rate.ceil() as u64)?;
    task.start()?;
    for _ in 0..10 {
        let data = task.read_sync(10_000)?;
        // [ai0 samples (LC), ai1 samples (Acc), ai2 samples (Mic)]
        assert_eq!(data.len() % 3, 0);
    }
    Ok(())
}
```

`read_sync` takes samples **per channel** and returns channel-grouped `Vec<f64>`
in registration order. It uses the actual sample count returned by DAQmx,
without padding. A zero request returns an empty vector, negative counts are
rejected. Reads wait up to 10 seconds. Driver errors, including timeouts with
partial data, return `Err`; positive DAQmx warning codes are treated as success.
Errors implement `Display` and `std::error::Error`, including the driver error
code, description and extended information.

Voltage channels use the device's default terminal configuration. Clock timing
uses the internal clock and rising edges. In continuous mode, the sample count
argument is used by DAQmx for buffer sizing. No actual-rate query is provided;
the device may coerce the requested sampling rate.

Construct and use a task on the acquisition thread. Tasks and their configuration
and channel objects share ownership of the driver handle and cannot be sent or
shared between threads. Dropping the last owner calls `DAQmxClearTask`, which
also stops a running task. The usual pattern of discarding channel return values
and dropping the task needs no explicit `stop()`. If channel objects are retained,
they must also be dropped to release the handle. `stop()` is available when an
explicit stop is needed. Cleanup errors in destructors are ignored.

Run the complete example with `cargo run --example continuous_voltage`.
It acquires ten blocks from the first enumerated device's ai0, ai1, ai2.

`cargo test` runs software tests and a driver task creation/cleanup test, which
requires the NI-DAQmx runtime but no DAQ device. To additionally test three-channel
100 kS/s acquisition with a real or simulated `Dev1` exposing ai0 through ai2:

```text
cargo test continuous_three_channel_acquisition -- --ignored
```

Hardware triggers, output, digital I/O, counters and asynchronous reads are not
implemented in the safe API.
