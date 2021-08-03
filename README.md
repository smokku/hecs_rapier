# physics engine _for hecs_

Port of `bevy_rapier2d` to hecs ECS.

## WARNING

- This code requires hecs fork with entity change tracking: <https://github.com/smokku/hecs.git>
- Unfinished. Joints handling is not yet ported.

## Example

```rust
use hecs_rapier as physics;
use hecs::World;
use resources::Resources;

let mut world = World::new();
let mut resources = Resources::new();

physics::systems::init(&mut world, &mut resources);

while running {
        physics::systems::attach_bodies_and_colliders(&mut world);
        // physics::systems::create_joints_system();
        physics::systems::finalize_collider_attach_to_bodies(&mut world, &resources);


        if time_to_step {

            physics::systems::step_world(&mut world, &resources);

        }


        physics::systems::collect_removals(&mut world, &resources);

        next_frame().await;
        world.clear_trackers();
}
```
