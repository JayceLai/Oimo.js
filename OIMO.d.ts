
// tslint:disable:max-line-length
// tslint:disable-next-line:no-namespace
declare namespace OIMO {
    const REVISION: '1.0.9';

    // BroadPhase
    const BR_NULL: 0;
    const BR_BRUTE_FORCE: 1;
    const BR_SWEEP_AND_PRUNE: 2;
    const BR_BOUNDING_VOLUME_TREE: 3;

    // Body type
    const BODY_NULL: 0;
    const BODY_DYNAMIC: 1;
    const BODY_STATIC: 2;
    const BODY_KINEMATIC: 3;
    const BODY_GHOST: 4;

    // Shape type
    const SHAPE_NULL: 0;
    const SHAPE_SPHERE: 1;
    const SHAPE_BOX: 2;
    const SHAPE_CYLINDER: 3;
    const SHAPE_PLANE: 4;
    const SHAPE_PARTICLE: 5;
    const SHAPE_TETRA: 6;

    // Joint type
    const JOINT_NULL: 0;
    const JOINT_DISTANCE: 1;
    const JOINT_BALL_AND_SOCKET: 2;
    const JOINT_HINGE: 3;
    const JOINT_WHEEL: 4;
    const JOINT_SLIDER: 5;
    const JOINT_PRISMATIC: 6;

    // AABB aproximation
    const AABB_PROX: 0.005;

    class Math {

    }

    class Vec3 {
        constructor (x?: number, y?: number, z?: number);
        x: number;
        y: number;
        z: number;
        Vec3: true;
        set (x, y, z): any;
        add (a, b): any;
        addVectors (a, b): any;
        addEqual (v): any;
        sub (a, b): any;
        subVectors (a, b): any;
        subEqual (v): any;
        scale (v, s): any;
        scaleEqual (s): any;
        multiply (v): any;
        scaleVectorEqual (v): any;
        addScaledVector (v, s): any;
        subScaledVector (v, s): any;
        addScale (v, s): any;
        subScale (v, s): any;
        cross (a, b): any;
        crossVectors (a, b): any;
        tangent (a): any;
        invert (v): any;
        negate (): any;
        dot (v): any;
        addition (): any;
        lengthSq (): any;
        length (): any;
        copy (v): any;
        mulMat (m, a): any;
        applyMatrix3 (m, transpose): any;
        applyQuaternion (q): any;
        testZero (): any;
        testDiff (v): any;
        equals (v): any;
        clone (): any;
        toString (): any;
        multiplyScalar (scalar): any;
        divideScalar (scalar): any;
        normalize (): any;
        toArray (array, offset): any;
        fromArray (array, offset): any;
    }

    class Quat {
        constructor (x?: number, y?: number, z?: number, w?: number);
        x: number;
        y: number;
        z: number;
        w: number;
        Quat: true;
        set (x, y, z, w): any;
        addTime (v, t): any;
        multiply (q, p): any;
        multiplyQuaternions (a, b): any;
        setFromUnitVectors (v1, v2): any;
        arc (v1, v2): any;
        normalize (): any;
        inverse (): any;
        invert (q): any;
        conjugate (): any;
        length (): any;
        lengthSq (): any;
        copy (q): any;
        clone (q): any;
        testDiff (q): any;
        equals (q): any;
        toString (): any;
        setFromEuler (x, y, z): any;
        setFromAxis (axis, rad): any;
        setFromMat33 (m): any;
        toArray (array, offset): any;
        fromArray (array, offset): any;
    }

    class Mat33 {
        constructor ();
        Mat33: true;
        set (e00, e01, e02, e10, e11, e12, e20, e21, e22): any;
        add (a, b): any;
        addMatrixs (a, b): any;
        addEqual (m): any;
        sub (a, b): any;
        subMatrixs (a, b): any;
        subEqual (m): any;
        scale (m, s): any;
        scaleEqual (s);// multiplyScala:anyr
        multiplyMatrices (m1, m2, transpose): any;
        transpose (m): any;
        setQuat (q): any;
        invert (m): any;
        addOffset (m, v): any;
        subOffset (m, v): any;
        multiplyScalar (s): any;
        identity (): any;
        clone (): any;
        copy (m): any;
        determinant (): any;
        fromArray (array, offset): any;
        toArray (array, offset): any;
    }

    class Shape {
        constructor (config);
        type;
        id;
        prev;
        next;
        proxy;
        parent;
        contactLink;
        numContacts;
        position;
        rotation;
        relativePosition;
        relativeRotation;
        aabb;
        density;
        friction;
        restitution;
        belongsTo;
        collidesWith;

        calculateMassInfo (out): any;
        updateProxy (): any;
    }

    class Box extends Shape {
        constructor (config, Width, Height, Depth);

        width;
        height;
        depth;

        halfWidth;
        halfHeight;
        halfDepth;

        dimentions;
        elements;
    }

    class Sphere extends Shape {
        constructor (config, radius);

        raidus;

        volume (): number;
    }

    class Cylinder extends Shape {
        constructor (config, radius, height);

        raidus;

        height;
        halfHeight;

        normalDirection;
        halfDirection;
    }

    class Plane extends Shape {
        constructor (config /**, normal */);

        normal: Vec3;

        volume (): number;
    }

    class Particle extends Shape {
        constructor (config /**, normal */);
        volume (): number;
    }

    interface ShapeConfig {
        // position of the shape in parent's coordinate system.
        relativePosition: Vec3;
        // rotation matrix of the shape in parent's coordinate system.
        relativeRotation: Mat33;
        // coefficient of friction of the shape.
        friction: number | 0.2; // 0.4
        // coefficient of restitution of the shape.
        restitution: number | 0.2;
        // density of the shape.
        density: number | 1;
        // bits of the collision groups to which the shape belongs.
        belongsTo: number | 1;
        // bits of the collision groups with which the shape collides.
        collidesWith: number | 0xffffffff;
    }

    interface MassInfo {
        // Mass of the shape.
        mass: number | 0;
        // The moment inertia of the shape.
        inertia: Mat33;
    }

    class RigidBody {
        constructor (Position?: Vec3, Rotation?: Quat);

        position: Vec3;
        orientation: Quat;

        scale: number | 1;
        invScale: number | 1;

        // possible link to three Mesh;
        mesh: any | null;

        id: any;
        name: string | "";
        // The maximum number of shapes that can be added to a one rigid.
        //MAX_SHAPES : 64;//64;

        prev: any | null;
        next: any | null;

        // I represent the kind of rigid body.
        // Please do not change from the outside this variable.
        // If you want to change the type of rigid body, always
        // Please specify the type you want to set the arguments of setupMass method.
        type: any; // BODY_NULL;

        massInfo: MassInfo;

        newPosition: Vec3;
        controlPos: boolean | false;
        newOrientation: Quat;
        newRotation: Vec3;
        currentRotation: Vec3;
        controlRot: boolean | false;
        controlRotInTime: boolean | false;

        quaternion: Quat;
        pos: Vec3;

        // Is the translational velocity.
        linearVelocity: Vec3;
        // Is the angular velocity.
        angularVelocity: Vec3;

        //--------------------------------------------
        //  Please do not change from the outside this variables.
        //--------------------------------------------

        // It is a world that rigid body has been added.
        parent: any | null;
        contactLink: any | null;
        numContacts: number | 0;

        // An array of shapes that are included in the rigid body.
        shapes: any | null;
        // The number of shapes that are included in the rigid body.
        numShapes: number | 0;

        // It is the link array of joint that is connected to the rigid body.
        jointLink: any | null;
        // The number of joints that are connected to the rigid body.
        numJoints: number | 0;

        // It is the world coordinate of the center of gravity in the sleep just before.
        sleepPosition: Vec3;
        // It is a quaternion that represents the attitude of sleep just before.
        sleepOrientation: Quat;
        // I will show this rigid body to determine whether it is a rigid body static.
        isStatic: boolean | false;
        // I indicates that this rigid body to determine whether it is a rigid body dynamic.
        isDynamic: boolean | false;

        isKinematic: boolean | false;

        // It is a rotation matrix representing the orientation.
        rotation: Mat33;

        //--------------------------------------------
        // It will be recalculated automatically from the shape, which is included.
        //--------------------------------------------

        // This is the weight.
        mass: number | 0;
        // It is the reciprocal of the mass.
        inverseMass: number | 0;
        // It is the inverse of the inertia tensor in the world system.
        inverseInertia: Mat33;
        // It is the inertia tensor in the initial state.
        localInertia: Mat33;
        // It is the inverse of the inertia tensor in the initial state.
        inverseLocalInertia: Mat33;

        tmpInertia: Mat33;


        // I indicates rigid body whether it has been added to the simulation Island.
        addedToIsland: boolean | false;
        // It shows how to sleep rigid body.
        allowSleep: boolean | true;
        // This is the time from when the rigid body at rest.
        sleepTime: number | 0;
        // I shows rigid body to determine whether it is a sleep state.
        sleeping: boolean | false;

        setParent (world): any;
        addShape (shape): any;
        removeShape (shape): any;
        remove (): any;
        dispose (): any;
        checkContact (name): any;
        setupMass (type, AdjustPosition): any;
        awake (): any;
        sleep (): any;
        testWakeUp (): any;
        isLonely (): any;
        updatePosition (timeStep): any;
        getAxis (): any;
        rotateInertia (rot, inertia, out): any;
        syncShapes (): any;
        applyImpulse (position, force): any;
        setPosition (pos): any;
        setQuaternion (q): any;
        setRotation (rot): any;
        resetPosition (x, y, z): any;
        resetQuaternion (q): any;
        resetRotation (x, y, z): any;
        getPosition (): any;
        getQuaternion (): any;
        connectMesh (mesh): any;
        updateMesh (): any;
    }

    class World {
        constructor (config);

        // this world scale defaut is 0.1 to 10 meters max for dynamique body
        scale: number | 1;
        invScale: number | 1;

        // The time between each step
        timeStep: number | 0.01666; // 1/60;
        timerate: number | 16.66;//timeStep * 1000;
        timer: any | null;

        preLoop: null;
        postLoop: null;

        // The number of iterations for constraint solvers.
        numIterations: number | 8;
        broadPhase: any; //BruteForceBroadPhase | SAPBroadPhase | DBVTBroadPhase;
        Btypes: ['None', 'BruteForce', 'Sweep & Prune', 'Bounding Volume Tree'];
        broadPhaseType: string | 'None' | 'BruteForce' | 'Sweep & Prune' | 'Bounding Volume Tree';

        // This is the detailed information of the performance.
        performance: any | null;
        isStat: any | false;

        /**
         * Whether the constraints randomizer is enabled or not.
         *
         * @property enableRandomizer
         * @type {Boolean}
         */
        enableRandomizer: any | true;

        // The rigid body list
        rigidBodies: any | null;
        // number of rigid body
        numRigidBodies: number | 0;
        // The contact list
        contacts: any | null;
        unusedContacts: any | null;
        // The number of contact
        numContacts: number | 0;
        // The number of contact points
        numContactPoints: number | 0;
        //  The joint list
        joints: any | null;
        // The number of joints.
        numJoints: number | 0;
        // The number of simulation islands.
        numIslands: number | 0;


        // The gravity in the world.
        gravity: Vec3;

        detectors: any[];
        // detectors[SHAPE_SPHERE][SHAPE_SPHERE]: new SphereSphereCollisionDetector();
        // detectors[SHAPE_SPHERE][SHAPE_BOX] : new SphereBoxCollisionDetector(false);
        // detectors[SHAPE_BOX][SHAPE_SPHERE] : new SphereBoxCollisionDetector(true);
        // detectors[SHAPE_BOX][SHAPE_BOX] : new BoxBoxCollisionDetector();

        //     // CYLINDER add
        // detectors[SHAPE_CYLINDER][SHAPE_CYLINDER] : new CylinderCylinderCollisionDetector();

        // detectors[SHAPE_CYLINDER][SHAPE_BOX] : new BoxCylinderCollisionDetector(true);
        // detectors[SHAPE_BOX][SHAPE_CYLINDER] : new BoxCylinderCollisionDetector(false);

        // detectors[SHAPE_CYLINDER][SHAPE_SPHERE] : new SphereCylinderCollisionDetector(true);
        // detectors[SHAPE_SPHERE][SHAPE_CYLINDER] : new SphereCylinderCollisionDetector(false);

        // // PLANE add

        // detectors[SHAPE_PLANE][SHAPE_SPHERE] : new SpherePlaneCollisionDetector(true);
        // detectors[SHAPE_SPHERE][SHAPE_PLANE] : new SpherePlaneCollisionDetector(false);

        // detectors[SHAPE_PLANE][SHAPE_BOX] : new BoxPlaneCollisionDetector(true);
        // detectors[SHAPE_BOX][SHAPE_PLANE] : new BoxPlaneCollisionDetector(false);

        // // TETRA add
        // //detectors[SHAPE_TETRA][SHAPE_TETRA] : new TetraTetraCollisionDetector();


        randX: number | 65535;
        randA: number | 98765;
        randB: number | 123456789;

        islandRigidBodies: any[];
        islandStack: any[];
        islandConstraints: any[];

        play (): any;
        stop (): any;
        setGravity (ar): any;
        getInfo (): any;
        getByName (name): any;
        removeShape (shape): any;
        addJoint (joint): any;
        removeJoint (joint): any;
        addContact (s1, s2): any;
        removeContact (contact): any;
        getContact (b1, b2): any;
        checkContact (name1, name2): any;
        callSleep (body): any;
        step (): any;
        // remove(obj):any;
        add (o): any;
        initBody (type, o): any;
        initJoint (type, o): any;
        addRigidBody (rigidBody): any;
        removeRigidBody (rigidBody): any;
    }
}

declare module '@cocos/oimo' {
    export = OIMO;
}
