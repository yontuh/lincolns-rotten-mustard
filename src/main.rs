use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use std::{
    f32::consts::{FRAC_PI_2, PI},
    ops::Range,
};

fn main() {
    App::new()
        .init_resource::<CameraSettings>()
        .init_resource::<CameraState>()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        // Calculates collisions at 240hz
        .insert_resource(Time::<Fixed>::from_seconds(1.0 / 2400.0))
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(Update, orbit)
        .add_systems(Update, reset_simulation)
        .add_systems(Update, move_robot)
        .run();
}

fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_scene_objects(&mut commands, &mut meshes, &mut materials);
}

const FTM: f32 = 0.3048;

const POUNDS_TO_KILOGRAMS: f32 = 0.453592;

const WALL_HEIGHT: f32 = 1.0 * FTM;

const WALL_LENGTH: f32 = 12.0 * FTM;

const ROBOT_LENGTH: f32 = 1.5 * FTM;

const BALL_RAD: f32 = ((5.0 / 12.0) / 2.0) * FTM;

const BALL_DENSITY: f32 = (0.165 * POUNDS_TO_KILOGRAMS)
    / ((4.0 / 3.0) * PI * (((2.5 / 12.0) * FTM) * ((2.5 / 12.0) * FTM) * ((2.5 / 12.0) * FTM)));

const ROBOT_DENSITY: f32 = (50.0 * POUNDS_TO_KILOGRAMS) / (1.0 * FTM) * (1.0 * FTM) * (1.0 * FTM);

const GOAL_HEIGHT: f32 = 0.9845;

fn spawn_scene_objects(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    println!("{}", ROBOT_DENSITY);
    // Arena
    commands.spawn((
        Collider::cuboid(WALL_LENGTH / 2.0, 0.001, WALL_LENGTH / 2.0),
        Transform::from_xyz(0.0, -0.001, 0.0),
        PhysicsObject,
        Mesh3d(meshes.add(Cuboid::new(
            WALL_LENGTH / 2.0 * 2.0,
            0.002,
            WALL_LENGTH / 2.0 * 2.0,
        ))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.8, 0.8))),
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

    // Robot
    commands.spawn((
        Velocity::default(),
        ExternalImpulse::default(),
        Ccd::enabled(),
        LockedAxes::ROTATION_LOCKED_X | LockedAxes::ROTATION_LOCKED_Z,
        RigidBody::Dynamic,
        Collider::cuboid(ROBOT_LENGTH / 2.0, 0.1, ROBOT_LENGTH / 2.0),
        Transform::from_xyz(0.0, 0.101, 0.0),
        PhysicsObject,
        ColliderMassProperties::Density(ROBOT_DENSITY),
        ReadMassProperties::default(),
        Friction::coefficient(0.0),
        Damping {
            linear_damping: 0.0,
            angular_damping: 0.0,
        },
        RobotObject {
            linear_speed: 1.50,
            turn_speed: 5.9,
            max_impulse: 0.5,
        },
        Mesh3d(meshes.add(Cuboid::new(ROBOT_LENGTH, 0.2, ROBOT_LENGTH))),
        MeshMaterial3d(materials.add(GREEN)),
    ));

    // Ball
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::ball(BALL_RAD))
        .insert(Ccd::enabled())
        .insert(Restitution::coefficient(0.5))
        .insert(ColliderMassProperties::Density(BALL_DENSITY))
        .insert(Damping {
            linear_damping: 0.1,
            angular_damping: 1.0,
        })
        .insert(Transform::from_xyz(0.0, 1.0, 0.0))
        .insert(PhysicsObject)
        .insert(Mesh3d(meshes.add(Sphere::new(BALL_RAD))))
        .insert(MeshMaterial3d(materials.add(WHITE)));
}

const WHITE: bevy::prelude::Color = Color::srgb(2.8, 2.8, 2.8);

const GREEN: bevy::prelude::Color = Color::srgb(0.0, 2.0, 0.0);

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

        // P - Control factor
        let stiffness = 0.05;

        let delta_lin = (target_linear_vel - velocity.linvel) * stiffness;
        let delta_ang = (target_angular_vel - velocity.angvel) * stiffness;

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
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if keys.just_pressed(KeyCode::KeyR) {
        for entity in &query {
            commands.entity(entity).despawn();
        }

        spawn_scene_objects(&mut commands, &mut meshes, &mut materials);
    }
}

#[derive(Component)]
struct PhysicsObject;

#[derive(Component)]
struct RobotObject {
    linear_speed: f32,
    turn_speed: f32,
    max_impulse: f32,
}

#[derive(Debug, Resource)]
struct CameraSettings {
    pub orbit_distance: f32,
    pub pitch_speed: f32,
    pub pitch_range: Range<f32>,
    pub yaw_speed: f32,
    pub pan_speed: f32,
}

fn setup_graphics(mut commands: Commands, camera_settings: Res<CameraSettings>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, -camera_settings.orbit_distance)
            .looking_at(Vec3::ZERO, Vec3::Y),
    ));
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
