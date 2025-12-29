use burn::{
    module::Module,
    nn::{Linear, LinearConfig, Relu},
    prelude::*,
};
use ipc_channel::ipc::IpcSender;
use serde::{Deserialize, Serialize};

#[derive(Module, Debug)]
pub struct Agent<B: Backend> {
    pub hidden_layer_1: Linear<B>, // First hidden layer, not the input 'layer'!
    pub hidden_layer_2: Linear<B>, // Second hidden layer
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
            // 2 Input, hidden_size (64) output(s)
            hidden_layer_1: LinearConfig::new(2, self.hidden_size).init(device),

            // Hidden: 64 -> 64 (The second hidden layer)
            hidden_layer_2: LinearConfig::new(self.hidden_size, self.hidden_size).init(device),

            // Output shape: 1
            output_layer: LinearConfig::new(self.hidden_size, 2).init(device),

            activation: Relu::new(),
        }
    }
}

impl<B: Backend> Agent<B> {
    pub fn forward(&self, input: Tensor<B, 2>) -> Tensor<B, 2> {
        // Forward information from the input 'layer'
        let x = self.hidden_layer_1.forward(input);
        let x = self.activation.forward(x);

        let x = self.hidden_layer_2.forward(x);
        let x = self.activation.forward(x);

        self.output_layer.forward(x)
    }
}

// -----------------------------------------------------------------------------------------------

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
