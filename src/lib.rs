use ipc_channel::ipc::IpcSender;
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct ModelChoice {
    pub yaw: f32,
    pub power: f32,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Reward {
    pub reward: f32,
}

#[derive(Serialize, Deserialize)]
pub struct Handshake {
    pub tx_choice: IpcSender<ModelChoice>,
    pub tx_back_channel: IpcSender<IpcSender<Reward>>,
}
