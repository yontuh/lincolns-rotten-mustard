use bevy::app::ScheduleRunnerPlugin;
use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use ipc_channel::ipc::{self, IpcSender};
use iyes_perf_ui::prelude::*;
use rand::prelude::*;
use shared::{Handshake, ModelChoice, ModelChoices, Poses, Reward, Rewards};
use std::env;
use std::sync::Mutex;
use std::time::Duration;
use std::{
    f32::consts::{FRAC_PI_2, PI},
    ops::Range,
};

fn main() {
    let headless = true;

    let mut app = App::new();

    if headless {
        app.add_plugins(
            MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(0.0))),
        );
    } else {
        app.add_plugins(DefaultPlugins)
            .add_plugins(bevy::diagnostic::FrameTimeDiagnosticsPlugin::default())
            .add_plugins(bevy::diagnostic::EntityCountDiagnosticsPlugin)
            .add_plugins(bevy::diagnostic::SystemInformationDiagnosticsPlugin)
            .add_plugins(bevy::render::diagnostic::RenderDiagnosticsPlugin)
            .add_plugins(PerfUiPlugin)
            .add_plugins(RapierDebugRenderPlugin::default())
            .add_systems(Startup, setup_graphics)
            .add_systems(FixedUpdate, move_robot)
            .add_systems(FixedUpdate, orbit)
            .add_systems(Update, reset_simulation);
    }

    app.init_resource::<CameraSettings>()
        .init_resource::<CameraState>()
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_systems(Startup, setup_physics_speed)
        .add_systems(Startup, setup_physics)
        .add_systems(Startup, setup_ipc)
        .add_systems(Update, run_training_loop)
        .run();
}

fn setup_physics_speed(mut timestep_mode: ResMut<TimestepMode>) {
    *timestep_mode = TimestepMode::Fixed {
        dt: 1.0 / 60.0,
        substeps: 1,
    };
}

fn setup_ipc(mut commands: Commands) {
    let args: Vec<String> = env::args().collect();
    let server_name = args.get(1).expect("provide name");

    let (tx_choice, rx_choice) = ipc::channel::<ModelChoices>().unwrap();
    let (tx_back_channel, rx_back_channel) = ipc::channel::<IpcSender<Rewards>>().unwrap();

    let tx_bootstrap: IpcSender<Handshake> = IpcSender::connect(server_name.to_string()).unwrap();

    tx_bootstrap
        .send(Handshake {
            tx_choice,
            tx_back_channel,
        })
        .unwrap();

    let tx_reward = rx_back_channel.recv().unwrap();

    commands.insert_resource(TrainingConnection {
        rx_choices: Mutex::new(rx_choice),
        tx_reward: Mutex::new(tx_reward),
        waiting_for_result: false,
        pending_setup: false,
        pending_launch: false,
        index: 0,
        pending_yaws: Vec::new(),
        pending_powers: Vec::new(),
        pending_x_pos: Vec::new(),
        pending_z_pos: Vec::new(),
        shot_frame_counter: 0,
        rewards: Rewards {
            rewards: Vec::new(),
        },
        batch_size: None,
    });
    println!("Done with IPC setup");
}

fn run_training_loop(
    mut commands: Commands,
    mut connection: ResMut<TrainingConnection>,
    mut take_out_query: Query<(&mut Transform, &mut RobotObject), Without<Ball>>,
    mut collision_events: EventReader<CollisionEvent>,
    reset_query: Query<Entity, With<ShouldReset>>,
    goal_query: Query<&GoalObject>,
    ball_query: Query<Entity, With<Ball>>,
    missed_query: Query<(Entity, &Transform), With<Ball>>,
) {
    if connection.pending_launch {
        take_out(
            &mut commands,
            &mut take_out_query,
            connection.pending_yaws[connection.index],
            connection.pending_powers[connection.index],
        );
        connection.pending_launch = false;
    }

    // Setup the Arena
    if connection.pending_setup {
        if let Ok((transform, _)) = take_out_query.single() {
            reset_with_pos(
                &mut commands,
                &reset_query,
                connection.pending_x_pos[connection.index],
                connection.pending_z_pos[connection.index],
            );
            println!(
                "[sim] Choice - Rotation: {:.2}, Power: {:.2} at Position: {}, {}",
                connection.pending_yaws[connection.index],
                connection.pending_powers[connection.index],
                transform.translation.x,
                transform.translation.z
            );
        }
        connection.pending_launch = true;
        connection.pending_setup = false;
        connection.waiting_for_result = true;
        return;
    }

    // Wait for and parse connection
    if !connection.waiting_for_result {
        // Halts until connection
        let received = connection.rx_choices.lock().unwrap().try_recv();

        match received {
            Ok(choices) => {
                println!(
                    "[sim] Received {:?} choices. Example yaws: {:?}, power: {:?} ",
                    choices.yaws.len(),
                    choices.yaws[0],
                    choices.powers[0]
                );

                println!("Recieved Choices: {:?}", choices.x_vec);

                connection.pending_yaws = Vec::new();
                connection.pending_powers = Vec::new();
                connection.pending_x_pos = Vec::new();
                connection.pending_z_pos = Vec::new();

                // Rewards is reset in the else clause

                for i in 0..choices.yaws.len() {
                    // reset_with_random_pos(&mut commands, &reset_query);

                    connection.pending_yaws.push(choices.yaws[i]);
                    connection.pending_powers.push(choices.powers[i]);
                    connection.pending_x_pos.push(choices.x_vec[i]);
                    connection.pending_z_pos.push(choices.z_vec[i]);
                }
                println!("Pending_x_poses: {:?}", connection.pending_x_pos);
                connection.pending_setup = true;
                connection.batch_size = Some(choices.yaws.len());

                connection.shot_frame_counter = 0;

                connection.index = 0;
            }
            Err(_) => {}
        }
    // Track simulation
    } else {
        connection.shot_frame_counter += 1;

        let success_reward = check_success(
            &mut commands,
            &mut collision_events,
            &goal_query,
            &ball_query,
        );
        let missed_reward = check_missed(&mut commands, &missed_query);

        if let Some(reward) = success_reward.or(missed_reward) {
            connection.rewards.rewards.push(reward.reward);
            connection.index += 1;
            println!("Index increased to: {}", connection.index);
            if connection.rewards.rewards.len() == connection.batch_size.unwrap() {
                println!("[sim] Sending Reward: {:?}", reward);
                // Replaces connection's rewards with Rewards{rewards: Vec::new()}
                // and returns the value that was previously in connection's rewards.
                let rewards_to_send = std::mem::replace(
                    &mut connection.rewards,
                    Rewards {
                        rewards: Vec::new(),
                    },
                );
                connection
                    .tx_reward
                    .lock()
                    .unwrap()
                    .send(rewards_to_send)
                    .unwrap();
                connection.waiting_for_result = false;
            } else {
                connection.pending_setup = true;
            }
        }
    }
}

const FTM: f32 = 0.3048;

const POUNDS_TO_KILOGRAMS: f32 = 0.453592;

const WALL_HEIGHT: f32 = 1.0 * FTM;

const WALL_LENGTH: f32 = 12.0 * FTM;

const ROBOT_LENGTH: f32 = (13.858 / 12.0) * FTM;

const ROBOT_HEIGHT: f32 = (6.0 / 12.0) * FTM;

const ROBOT_WEIGHT: f32 = 20.0 * POUNDS_TO_KILOGRAMS;

const BALL_RAD: f32 = ((5.0 / 12.0) / 2.0) * FTM;

const BALL_WEIGHT: f32 = 0.165 * POUNDS_TO_KILOGRAMS;

const GOAL_HEIGHT: f32 = 0.9845;

fn spawn_arena_objects(commands: &mut Commands) {
    commands.spawn((
        Collider::cuboid(WALL_LENGTH / 2.0, 0.001, WALL_LENGTH / 2.0),
        Transform::from_xyz(0.0, -0.001, 0.0),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(0.001, WALL_HEIGHT / 2.0, WALL_LENGTH / 2.0),
        Transform::from_xyz(WALL_LENGTH / 2.0, WALL_HEIGHT / 2.0, 0.0),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(0.001, WALL_HEIGHT / 2.0, WALL_LENGTH / 2.0),
        Transform::from_xyz(-WALL_LENGTH / 2.0, WALL_HEIGHT / 2.0, 0.0),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(WALL_LENGTH / 2.0, WALL_HEIGHT / 2.0, 0.001),
        Transform::from_xyz(0.0, WALL_HEIGHT / 2.0, WALL_LENGTH / 2.0),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(WALL_LENGTH / 2.0, WALL_HEIGHT / 2.0, 0.001),
        Transform::from_xyz(0.0, WALL_HEIGHT / 2.0, -WALL_LENGTH / 2.0),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(0.495, GOAL_HEIGHT / 2.0, 0.001),
        Transform::from_xyz(1.479, GOAL_HEIGHT / 2.0, 1.479).with_rotation(Quat::from_euler(
            EulerRot::YXZ,
            PI / 4.0,
            0.0,
            0.0,
        )),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(0.35, GOAL_HEIGHT / 2.0, 0.001),
        Transform::from_xyz(
            (WALL_LENGTH / 2.0) - 0.35,
            GOAL_HEIGHT / 2.0,
            WALL_LENGTH / 2.0,
        ),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::triangle(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.70, 0.0, 0.0),
            Vec3::new(0.70, 0.375, 0.0),
        ),
        Transform::from_xyz((WALL_LENGTH / 2.0) - 0.7, GOAL_HEIGHT, WALL_LENGTH / 2.0),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(0.001, GOAL_HEIGHT / 2.0, 0.35),
        Transform::from_xyz(
            WALL_LENGTH / 2.0,
            GOAL_HEIGHT / 2.0,
            (WALL_LENGTH / 2.0) - 0.35,
        ),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::triangle(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.70),
            Vec3::new(0.0, 0.375, 0.70),
        ),
        Transform::from_xyz(WALL_LENGTH / 2.0, GOAL_HEIGHT, (WALL_LENGTH / 2.0) - 0.7),
        PhysicsObject,
    ));

    commands.spawn((
        Sensor,
        ActiveEvents::COLLISION_EVENTS,
        Collider::triangle(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.7),
            Vec3::new(-0.7, 0.0, 0.7),
        ),
        Transform::from_xyz(
            WALL_LENGTH / 2.0,
            GOAL_HEIGHT - 0.2,
            (WALL_LENGTH / 2.0) - 0.7,
        ),
        GoalObject,
    ));
}

fn spawn_robot_objects(pos: Transform, commands: &mut Commands) {
    // Robot
    commands
        .spawn((
            Velocity::default(),
            ExternalImpulse::default(),
            Ccd::enabled(),
            LockedAxes::ROTATION_LOCKED_X | LockedAxes::ROTATION_LOCKED_Z,
            RigidBody::Dynamic,
            Collider::cuboid(ROBOT_LENGTH / 2.0, ROBOT_HEIGHT / 2.0, ROBOT_LENGTH / 2.0),
            pos,
            ColliderMassProperties::Mass(ROBOT_WEIGHT),
            ReadMassProperties::default(),
            Friction::coefficient(0.0),
            Damping {
                linear_damping: 0.0,
                angular_damping: 0.0,
            },
            RobotObject {
                linear_speed: 10.50,
                turn_speed: 3.5,
                max_impulse: 0.75, // "Torque"
                snozzle_angle: 80.0,
                snozzle_pow: 6.5, // m/s,
                stiffness: 0.5,
            },
            CollisionGroups::new(Group::GROUP_1, Group::ALL ^ Group::GROUP_2),
        ))
        .insert((PhysicsObject, DebugObject, ShouldReset));
}

fn take_out(
    commands: &mut Commands,
    query: &mut Query<(&mut Transform, &mut RobotObject), Without<Ball>>,
    yaw: f32,
    power: f32,
) {
    for (mut transform, mut robot) in query {
        robot.snozzle_pow = power;

        transform.rotation = Quat::from_rotation_y(yaw.to_radians());

        let pitch_radians = -robot.snozzle_angle.to_radians();

        let pitch_rotation = Quat::from_rotation_x(pitch_radians);

        let final_rotation = transform.rotation * pitch_rotation;

        let forward_dir = final_rotation * Vec3::Z;
        let shoot_velocity = forward_dir * robot.snozzle_pow;

        let spawn_pos = transform.translation + (Vec3::Y * ROBOT_HEIGHT);

        commands.spawn(BallBundle::new(spawn_pos, shoot_velocity));
    }
}

fn check_success(
    commands: &mut Commands,
    collision_events: &mut EventReader<CollisionEvent>,
    goal_query: &Query<&GoalObject>,
    ball_query: &Query<Entity, With<Ball>>,
) -> Option<Reward> {
    for event in collision_events.read() {
        match event {
            CollisionEvent::Started(entity1, entity2, _flags) => {
                let ball_entity = if goal_query.contains(*entity1) && ball_query.contains(*entity2)
                {
                    Some(*entity2)
                } else if goal_query.contains(*entity2) && ball_query.contains(*entity1) {
                    Some(*entity1)
                } else {
                    None
                };

                if let Some(entity) = ball_entity {
                    println!("✅ goaled ✅✅✅✅✅✅✅");
                    commands.entity(entity).despawn();
                    return Some(Reward { reward: 1.0 });
                }
            }
            _ => {}
        }
    }
    None
}

fn check_missed(
    commands: &mut Commands,
    query: &Query<(Entity, &Transform), With<Ball>>,
) -> Option<Reward> {
    for (entity, transform) in query {
        if transform.translation.y <= BALL_RAD + 0.05 {
            println!("missed");
            commands.entity(entity).despawn();
            return Some(Reward { reward: 0.0 });
        }
    }
    None
}

fn move_robot(
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<(
        &mut ExternalImpulse,
        &Velocity,
        &Transform,
        &RobotObject,
        &ReadMassProperties,
    )>,
) {
    for (mut impulse, velocity, transform, robot, mass_props) in &mut query {
        let mut linear_move = Vec3::ZERO;
        let mut turn_move = 0.0;
        if keys.pressed(KeyCode::KeyI) {
            linear_move.z += 1.0;
        }
        if keys.pressed(KeyCode::KeyK) {
            linear_move.z -= 1.0;
        }
        if keys.pressed(KeyCode::KeyJ) {
            linear_move.x += 1.0;
        }
        if keys.pressed(KeyCode::KeyL) {
            linear_move.x -= 1.0;
        }
        if keys.pressed(KeyCode::KeyZ) {
            turn_move += 1.0;
        }
        if keys.pressed(KeyCode::KeyX) {
            turn_move -= 1.0;
        }

        if linear_move.length_squared() > 0.0 {
            linear_move = linear_move.normalize();
        }

        let target_linear_vel = transform.rotation * linear_move * robot.linear_speed;

        let target_angular_vel = Vec3::new(0.0, turn_move * robot.turn_speed, 0.0);

        let delta_lin = (target_linear_vel - velocity.linvel) * robot.stiffness;
        let delta_ang = (target_angular_vel - velocity.angvel) * robot.stiffness;

        // Doesn't affect y-axis!
        let raw_impulse = Vec3::new(delta_lin.x, 0.0, delta_lin.z) * mass_props.mass;

        impulse.impulse = raw_impulse.clamp_length_max(robot.max_impulse);
        impulse.torque_impulse = delta_ang * mass_props.principal_inertia;
    }
}

fn reset_simulation(
    mut commands: Commands,
    keys: Res<ButtonInput<KeyCode>>,
    query: Query<Entity, With<PhysicsObject>>,
) {
    if keys.just_pressed(KeyCode::KeyR) {
        for entity in &query {
            commands.entity(entity).despawn();
        }

        spawn_arena_objects(&mut commands);

        spawn_robot_objects(
            Transform::from_xyz(0.0, 0.101, 0.0).with_rotation(Quat::from_euler(
                EulerRot::YXZ,
                PI / 4.0,
                0.0,
                0.0,
            )),
            &mut commands,
        );
    }
}

fn reset_with_pos(
    commands: &mut Commands,
    query: &Query<Entity, With<ShouldReset>>,
    x_feet: f32,
    z_feet: f32,
) {
    for entity in query {
        commands.entity(entity).despawn();
    }

    // Feet!
    if x_feet > 3.0 && z_feet > 3.0 {
        println!("Something has gone wrong, reset_with_pos()");
    }
    let spawn_pos = Transform::from_xyz(x_feet * FTM, 0.070, z_feet * FTM)
        .with_rotation(Quat::from_euler(EulerRot::YXZ, 0.0, 0.0, 0.0));

    spawn_robot_objects(spawn_pos, commands);
}

#[derive(Component)]
struct ShouldReset;

#[derive(Component)]
struct PhysicsObject;

#[derive(Component)]
struct GoalObject;

#[derive(Component)]
struct DebugObject;

#[derive(Component)]
struct RobotObject {
    linear_speed: f32,
    turn_speed: f32,
    max_impulse: f32,
    snozzle_angle: f32,
    snozzle_pow: f32,
    stiffness: f32,
}

#[derive(Component)]
struct Ball;

// Bevy only supports 15 for tuples, not structs
#[derive(Bundle)]
struct BallBundle {
    rigid_body: RigidBody,
    velocity: Velocity,
    collider: Collider,
    ccd: Ccd,
    restitution: Restitution,
    mass_props: ColliderMassProperties,
    read_mass: ReadMassProperties,
    damping: Damping,
    physics_object: PhysicsObject,
    debug_object: DebugObject,
    transform: Transform,
    ball: Ball,
    resettable: ShouldReset,
    collision_groups: CollisionGroups,
}

impl BallBundle {
    fn new(position: Vec3, linear_velocity: Vec3) -> Self {
        Self {
            rigid_body: RigidBody::Dynamic,
            velocity: Velocity {
                linvel: linear_velocity,
                angvel: Vec3::new(0.2, 0.4, 0.8),
            },
            collider: Collider::ball(BALL_RAD),
            ccd: Ccd::enabled(),
            restitution: Restitution::coefficient(0.5),
            mass_props: ColliderMassProperties::Mass(BALL_WEIGHT),
            read_mass: ReadMassProperties::default(),
            damping: Damping {
                linear_damping: 0.1,
                angular_damping: 1.0,
            },
            physics_object: PhysicsObject,
            debug_object: DebugObject,
            transform: Transform::from_translation(position),
            ball: Ball,
            resettable: ShouldReset,
            collision_groups: CollisionGroups::new(Group::GROUP_2, Group::ALL ^ Group::GROUP_1),
        }
    }
}

#[derive(Debug, Resource)]
struct CameraSettings {
    pub orbit_distance: f32,
    pub pitch_speed: f32,
    pub pitch_range: Range<f32>,
    pub yaw_speed: f32,
    pub pan_speed: f32,
}

#[derive(Resource)]
struct TrainingConnection {
    rx_choices: Mutex<ipc::IpcReceiver<ModelChoices>>,
    tx_reward: Mutex<IpcSender<Rewards>>,
    waiting_for_result: bool,
    pending_setup: bool,
    pending_launch: bool,
    index: usize,
    pending_yaws: Vec<f32>,
    pending_powers: Vec<f32>,
    pending_x_pos: Vec<f32>,
    pending_z_pos: Vec<f32>,
    shot_frame_counter: usize,
    rewards: Rewards,
    batch_size: Option<usize>,
}

fn setup_graphics(mut commands: Commands, camera_settings: Res<CameraSettings>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, -camera_settings.orbit_distance)
            .looking_at(Vec3::ZERO, Vec3::Y),
    ));
    // commands.spawn(PerfUiAllEntries::default());
}

fn setup_physics(mut commands: Commands) {
    spawn_arena_objects(&mut commands);
    spawn_robot_objects(
        Transform::from_xyz(0.0, 0.101, 0.0).with_rotation(Quat::from_euler(
            EulerRot::YXZ,
            PI / 4.0,
            0.0,
            0.0,
        )),
        &mut commands,
    );
}
impl Default for CameraSettings {
    fn default() -> Self {
        let pitch_limit = FRAC_PI_2 - 0.01;
        Self {
            orbit_distance: 5.0,
            pitch_speed: 0.003,
            pitch_range: -pitch_limit..pitch_limit,
            yaw_speed: 0.004,
            pan_speed: 0.005,
        }
    }
}

// Center of panning
#[derive(Debug, Resource, Default)]
struct CameraState {
    pub focus_point: Vec3,
}

fn orbit(
    mut camera: Single<&mut Transform, With<Camera>>,
    camera_settings: Res<CameraSettings>,
    mut camera_state: ResMut<CameraState>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
) {
    let delta = mouse_motion.delta;

    if delta == Vec2::ZERO {
        return;
    }

    if mouse_buttons.pressed(MouseButton::Left) {
        let delta_pitch = delta.y * camera_settings.pitch_speed;
        let delta_yaw = delta.x * camera_settings.yaw_speed;

        let (yaw, pitch, _roll) = camera.rotation.to_euler(EulerRot::YXZ);

        let new_pitch = (pitch + delta_pitch).clamp(
            camera_settings.pitch_range.start,
            camera_settings.pitch_range.end,
        );
        let new_yaw = yaw + delta_yaw;

        camera.rotation = Quat::from_euler(EulerRot::YXZ, new_yaw, new_pitch, 0.0);
    } else if mouse_buttons.pressed(MouseButton::Right) {
        let right = camera.right();
        let up = camera.up();

        let pan_vector = (right * -delta.x * camera_settings.pan_speed)
            + (up * delta.y * camera_settings.pan_speed);

        camera_state.focus_point += pan_vector;
    }

    camera.translation =
        camera_state.focus_point - camera.forward() * camera_settings.orbit_distance;
}
