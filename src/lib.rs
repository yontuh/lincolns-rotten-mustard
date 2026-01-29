use burn::{
    module::Module,
    nn::{Linear, LinearConfig, Relu},
    prelude::*,
};
use ipc_channel::ipc::IpcSender;
use serde::{Deserialize, Serialize};

// --- JNI Imports ---
#[cfg(any(target_os = "android", target_os = "linux"))]
use burn::backend::NdArray;
#[cfg(any(target_os = "android", target_os = "linux"))]
use burn::record::{BinBytesRecorder, FullPrecisionSettings, Recorder};
#[cfg(any(target_os = "android", target_os = "linux"))]
use jni::objects::JClass; // Removed unused JObject, JValue
#[cfg(any(target_os = "android", target_os = "linux"))]
use jni::sys::{jfloat, jfloatArray};
#[cfg(any(target_os = "android", target_os = "linux"))]
use jni::JNIEnv;
#[cfg(any(target_os = "android", target_os = "linux"))]
use std::sync::{Mutex, OnceLock}; // Added Mutex

// =========================================================================
//  MODEL DEFINITIONS
// =========================================================================

#[derive(Module, Debug)]
pub struct Agent<B: Backend> {
    pub hidden_layer_1: Linear<B>,
    pub hidden_layer_2: Linear<B>,
    pub output_layer: Linear<B>,
    pub activation: Relu,
}

#[derive(Config, Debug)]
pub struct AgentConfig {
    pub hidden_size: usize,
}

impl AgentConfig {
    pub fn init<B: Backend>(&self, device: &B::Device) -> Agent<B> {
        Agent {
            hidden_layer_1: LinearConfig::new(2, self.hidden_size).init(device),
            hidden_layer_2: LinearConfig::new(self.hidden_size, self.hidden_size).init(device),
            output_layer: LinearConfig::new(self.hidden_size, 2).init(device),
            activation: Relu::new(),
        }
    }
}

impl<B: Backend> Agent<B> {
    pub fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2> {
        let x = self.hidden_layer_1.forward(input);
        let x = self.activation.forward(x);

        let x = self.hidden_layer_2.forward(x);
        let x = self.activation.forward(x);

        self.output_layer.forward(x)
    }
}

// =========================================================================
//  IPC / SHARED DATA STRUCTURES
// =========================================================================

#[derive(Debug, Serialize, Deserialize)]
pub struct ModelChoice {
    pub yaw: f32,
    pub power: f32,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ModelChoices {
    pub yaws: Vec<f32>,
    pub powers: Vec<f32>,
    pub x_vec: Vec<f32>,
    pub z_vec: Vec<f32>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Pos {
    x: f32,
    z: f32,
}
#[derive(Debug, Serialize, Deserialize)]
pub struct Poses {
    pub x_vec: Vec<f32>,
    pub z_vec: Vec<f32>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Reward {
    pub reward: f32,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Rewards {
    pub rewards: Vec<f32>,
}

#[derive(Serialize, Deserialize)]
pub struct Handshake {
    pub tx_choice: IpcSender<ModelChoices>,
    pub tx_back_channel: IpcSender<IpcSender<Rewards>>,
}

// =========================================================================
//  linu JNI BRIDGE
#[cfg(any(target_os = "android", target_os = "linux"))]
#[cfg(any(target_os = "android", target_os = "linux"))]
static CONFIG_JSON: &str = include_str!("/tmp/rotten_mustard/config.json");
#[cfg(any(target_os = "android", target_os = "linux"))]
static MODEL_BYTES: &[u8] = include_bytes!("/tmp/rotten_mustard/model.bin");

#[cfg(any(target_os = "android", target_os = "linux"))]
type InferenceBackend = NdArray<f32>;

// WRAPPER FIX: Wrapped Agent in Mutex to make it Sync
#[cfg(any(target_os = "android", target_os = "linux"))]
static MODEL: OnceLock<Mutex<Agent<InferenceBackend>>> = OnceLock::new();

#[cfg(any(target_os = "android", target_os = "linux"))]
fn get_model() -> &'static Mutex<Agent<InferenceBackend>> {
    MODEL.get_or_init(|| {
        let device = Default::default();
        let config: AgentConfig =
            serde_json::from_str(CONFIG_JSON).expect("Failed to parse embedded config");

        let record = BinBytesRecorder::<FullPrecisionSettings>::default()
            .load(MODEL_BYTES.to_vec(), &device)
            .expect("Failed to load embedded model weights");

        let agent = config.init(&device).load_record(record);
        Mutex::new(agent)
    })
}

#[cfg(target_os = "android")]
#[unsafe(no_mangle)]
pub extern "system" fn Java_org_firstinspires_ftc_teamcode_Bambusa_RottenMustard_LincolnsRottenMustard_runInference(
    env: JNIEnv,
    _class: JClass,
    x: jfloat,
    z: jfloat,
) -> jfloatArray {
    let model_mutex = get_model();
    let model = model_mutex.lock().unwrap();
    let device = Default::default();

    let input_vec = vec![x as f32, z as f32];
    let input_tensor =
        Tensor::<InferenceBackend, 1>::from_floats(input_vec.as_slice(), &device).reshape([1, 2]);

    let output = model.forward(input_tensor);
    let output_vec: Vec<f32> = output.to_data().to_vec().unwrap();

    let output_array = env.new_float_array(output_vec.len() as i32).unwrap();
    env.set_float_array_region(&output_array, 0, &output_vec)
        .unwrap();

    output_array.into_raw()
}
