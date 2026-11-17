//! Synchronous NI-DAQmx analog voltage acquisition.
mod nidaqmx_sys;

use nidaqmx_sys as sys;
use std::{ffi::CString, fmt, ptr, rc::Rc};

/// An NI-DAQmx error, or invalid input (code 0).
#[derive(Debug)]
pub struct Error {
    pub code: i32,
    pub description: String,
    pub extended_info: String,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "NI-DAQmx ({}): {}", self.code, self.description)?;
        if !self.extended_info.is_empty() {
            write!(f, ": {}", self.extended_info)?;
        }

        Ok(())
    }
}

impl std::error::Error for Error {}

fn invalid(message: &str) -> Error {
    Error {
        code: 0,
        description: message.into(),
        extended_info: String::new(),
    }
}

fn cstring(value: &str) -> Result<CString, Error> {
    CString::new(value).map_err(|_| invalid("String contains an interior NUL byte"))
}

fn decode(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes.split(|&b| b == 0).next().unwrap_or_default()).into_owned()
}

fn check(code: i32) -> Result<(), Error> {
    if code >= 0 {
        return Ok(());
    }

    // Capture extended information before another DAQmx call can replace it.
    let mut extended = vec![0u8; 65536];
    let mut description = vec![0u8; 65536];
    unsafe {
        sys::DAQmxGetExtendedErrorInfo(extended.as_mut_ptr().cast(), extended.len() as u32);
        sys::DAQmxGetErrorString(
            code,
            description.as_mut_ptr().cast(),
            description.len() as u32,
        );
    }

    Err(Error {
        code,
        description: decode(&description),
        extended_info: decode(&extended),
    })
}

// DAQmx string getters return the required buffer size for a zero-sized query.
fn get_string(mut get: impl FnMut(*mut std::os::raw::c_char, u32) -> i32) -> Result<String, Error> {
    loop {
        let size = get(ptr::null_mut(), 0);
        check(size)?;
        if size == 0 {
            return Ok(String::new());
        }

        let mut bytes = vec![0u8; size as usize];
        let code = get(bytes.as_mut_ptr().cast(), size as u32);
        if code == sys::DAQmxErrorBufferTooSmallForString
            || code == sys::DAQmxWarningCAPIStringTruncatedToFitBuffer as i32
        {
            continue;
        }
        check(code)?;

        return Ok(decode(&bytes));
    }
}

fn names(value: &str) -> Vec<String> {
    value
        .split(',')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(str::to_owned)
        .collect()
}

/// Enumerate installed DAQ devices. No devices is an empty list.
pub fn get_system_devices() -> Result<Vec<String>, Error> {
    get_string(|buf, size| unsafe { sys::DAQmxGetSysDevNames(buf, size) }).map(|s| names(&s))
}

struct TaskHandle(sys::TaskHandle);

impl Drop for TaskHandle {
    fn drop(&mut self) {
        // Clear also stops a running task. Destructors cannot report driver errors.
        unsafe {
            sys::DAQmxClearTask(self.0);
        }
    }
}

/// A task and its configuration. Create and use it on the acquisition thread.
/// Dropping the last task/channel/configuration owner stops and clears the task.
pub struct Task {
    handle: Rc<TaskHandle>,
    pub ai_channels: AIChannelCollection,
    pub timing: Timing,
}

pub struct AIChannelCollection {
    handle: Rc<TaskHandle>,
}

pub struct AIChannel {
    _handle: Rc<TaskHandle>,
    pub name: String,
}

pub struct Timing {
    handle: Rc<TaskHandle>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleMode {
    FiniteSamps,
    ContinuousSamps,
}

impl Task {
    pub fn new(name: &str) -> Result<Self, Error> {
        let name = cstring(name)?;
        let mut raw = ptr::null_mut();
        check(unsafe { sys::DAQmxCreateTask(name.as_ptr(), &mut raw) })?;

        let handle = Rc::new(TaskHandle(raw));

        Ok(Self {
            ai_channels: AIChannelCollection {
                handle: handle.clone(),
            },
            timing: Timing {
                handle: handle.clone(),
            },
            handle,
        })
    }

    pub fn start(&self) -> Result<(), Error> {
        check(unsafe { sys::DAQmxStartTask(self.handle.0) })
    }

    pub fn stop(&self) -> Result<(), Error> {
        check(unsafe { sys::DAQmxStopTask(self.handle.0) })
    }

    /// Read samples per channel with a 10 second timeout, grouped by channel in
    /// registration order. Returns only samples actually read; driver errors
    /// (including timeouts) are returned even if the driver read partial data.
    /// A zero request returns an empty vector; negative requests are rejected.
    pub fn read_sync(&self, samples: i32) -> Result<Vec<f64>, Error> {
        if samples < 0 {
            return Err(invalid("Sample count must be nonnegative"));
        }
        if samples == 0 {
            return Ok(Vec::new());
        }

        let mut channels = 0;
        check(unsafe { sys::DAQmxGetTaskNumChans(self.handle.0, &mut channels) })?;
        if channels == 0 {
            return Err(invalid("Task has no channels"));
        }

        let len = (samples as usize)
            .checked_mul(channels as usize)
            .filter(|&n| n <= u32::MAX as usize)
            .ok_or_else(|| invalid("Read buffer is too large"))?;
        let mut data = Vec::new();
        data.try_reserve_exact(len)
            .map_err(|_| invalid("Cannot allocate read buffer"))?;
        data.resize(len, 0.0);

        let mut read = 0;
        check(unsafe {
            sys::DAQmxReadAnalogF64(
                self.handle.0,
                samples,
                10.0,
                sys::DAQmx_Val_GroupByChannel,
                data.as_mut_ptr(),
                len as u32,
                &mut read,
                ptr::null_mut(),
            )
        })?;

        compact_samples(data, channels as usize, samples as usize, read)
    }
}

// GroupByChannel uses the requested buffer width for each channel, as in NI's
// nidaqmx-python Task.read. Remove unused tails from each channel separately.
fn compact_samples(
    mut data: Vec<f64>,
    channels: usize,
    requested: usize,
    read: i32,
) -> Result<Vec<f64>, Error> {
    if read < 0 || read as usize > requested {
        return Err(invalid("Driver returned an invalid sample count"));
    }

    let read = read as usize;
    if read < requested {
        for channel in 1..channels {
            data.copy_within(
                channel * requested..channel * requested + read,
                channel * read,
            );
        }
    }

    data.truncate(read * channels);
    Ok(data)
}

impl Timing {
    /// Configure the internal sample clock on rising edges. In continuous mode,
    /// samples_per_channel determines the driver's buffer sizing.
    pub fn cfg_samp_clk_timing(
        &self,
        rate: f64,
        mode: SampleMode,
        samples_per_channel: u64,
    ) -> Result<(), Error> {
        if !rate.is_finite() || rate <= 0.0 || samples_per_channel == 0 {
            return Err(invalid(
                "Rate must be positive and finite, and sample count must be positive",
            ));
        }

        let mode = match mode {
            SampleMode::FiniteSamps => sys::DAQmx_Val_FiniteSamps,
            SampleMode::ContinuousSamps => sys::DAQmx_Val_ContSamps,
        };

        check(unsafe {
            sys::DAQmxCfgSampClkTiming(
                self.handle.0,
                c"".as_ptr(),
                rate,
                sys::DAQmx_Val_Rising as i32,
                mode as i32,
                samples_per_channel,
            )
        })
    }
}

impl AIChannelCollection {
    /// Add voltage input(s) using the device's default terminal configuration.
    pub fn add_ai_voltage_chan(
        &self,
        physical_channel: &str,
        min_val: f64,
        max_val: f64,
    ) -> Result<AIChannel, Error> {
        let physical = cstring(physical_channel)?;
        check(unsafe {
            sys::DAQmxCreateAIVoltageChan(
                self.handle.0,
                physical.as_ptr(),
                c"".as_ptr(),
                sys::DAQmx_Val_Default,
                min_val,
                max_val,
                sys::DAQmx_Val_Volts as i32,
                ptr::null(),
            )
        })?;

        Ok(AIChannel {
            _handle: self.handle.clone(),
            name: physical_channel.into(),
        })
    }

    pub fn channel_names(&self) -> Result<Vec<String>, Error> {
        get_string(|buf, size| unsafe { sys::DAQmxGetTaskChannels(self.handle.0, buf, size) })
            .map(|s| names(&s))
    }

    pub fn channel_at(&self, index: usize) -> Result<AIChannel, Error> {
        let name = self
            .channel_names()?
            .get(index)
            .cloned()
            .ok_or_else(|| invalid("Channel index out of bounds"))?;

        Ok(AIChannel {
            _handle: self.handle.clone(),
            name,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_empty_and_ordered_names() {
        assert!(names("").is_empty());
        assert_eq!(
            names("Dev1/ai0, Dev1/ai1, Dev1/ai2"),
            ["Dev1/ai0", "Dev1/ai1", "Dev1/ai2"]
        );
    }

    #[test]
    fn partial_and_empty_reads_preserve_channel_layout() {
        assert_eq!(
            compact_samples(vec![1., 2., 0., 10., 20., 0., 100., 200., 0.], 3, 3, 2).unwrap(),
            [1., 2., 10., 20., 100., 200.]
        );
        assert!(compact_samples(vec![0.; 9], 3, 3, 0).unwrap().is_empty());
        assert!(compact_samples(vec![0.; 9], 3, 3, 4).is_err());
        assert!(compact_samples(vec![0.; 9], 3, 3, -1).is_err());
    }

    #[test]
    fn nul_input_is_a_displayable_error() {
        let error = Task::new("bad\0name").err().unwrap();
        assert!(error.to_string().contains("NUL"));
    }

    #[test]
    #[ignore = "requires NI-DAQmx runtime and Dev1 with three analog inputs"]
    fn continuous_three_channel_acquisition() {
        let task = Task::new("").unwrap();
        for i in 0..3 {
            task.ai_channels
                .add_ai_voltage_chan(&format!("Dev1/ai{i}"), -10., 10.)
                .unwrap();
        }
        task.timing
            .cfg_samp_clk_timing(100_000., SampleMode::ContinuousSamps, 100_000)
            .unwrap();
        task.start().unwrap();
        for _ in 0..3 {
            assert_eq!(task.read_sync(10_000).unwrap().len(), 30_000);
        }
        // Drop must release the running task without stop().
    }
}
