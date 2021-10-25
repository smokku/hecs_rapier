use hecs::Entity;
use rapier2d::dynamics::{JointHandle, JointParams};

/// A component representing a joint added to the JointSet resource.
///
/// This component should not be created manually. It is automatically created and
/// added to an entity by the `JointBuilderComponent`.
pub struct JointHandleComponent {
    handle: JointHandle,
    entity1: Entity,
    entity2: Entity,
}

impl JointHandleComponent {
    pub(crate) fn new(handle: JointHandle, entity1: Entity, entity2: Entity) -> Self {
        Self {
            handle,
            entity1,
            entity2,
        }
    }

    /// The Rapier handle of the joint.
    pub fn handle(&self) -> JointHandle {
        self.handle
    }

    /// The first Bevy entity affected by this joint.
    pub fn entity1(&self) -> Entity {
        self.entity1
    }

    /// The second Bevy entity affected by this joint.
    pub fn entity2(&self) -> Entity {
        self.entity2
    }
}

/// Component responsible for initializing a Rapier joint.
///
/// This is a transient component that will be automatically replaced by a `JointHandleComponent`
/// once the Rapier joint it describes has been created and added to the `JointSet` resource.
pub struct JointBuilderComponent {
    pub(crate) params: JointParams,
    pub(crate) entity1: Entity,
    pub(crate) entity2: Entity,
}

impl JointBuilderComponent {
    /// Initializes a joint builder from the given joint params and the entities attached to this joint.
    pub fn new<J>(joint: J, entity1: Entity, entity2: Entity) -> Self
    where
        J: Into<JointParams>,
    {
        JointBuilderComponent {
            params: joint.into(),
            entity1,
            entity2,
        }
    }
}
