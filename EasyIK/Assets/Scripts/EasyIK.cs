
public class EasyIK : Component, Component.ExecuteInEditor
{
	[Property] public int numberOfJoints { get; set; } = 2;
	[Property] public GameObject ikTarget { get; set; }
	[Property] public int iterations { get; set; } = 10;
	[Property] public float tolerance { get; set; } = 0.05f;
	private GameObject[] jointTransforms { get; set; }
    private Vector3 startPosition { get; set; }
	[Property] private Vector3[] jointPositions { get; set; }
    private float[] boneLength { get; set; }
    private float jointChainLength { get; set; }
    private float distanceToTarget { get; set; }
    private Rotation[] startRotation { get; set; }
    private Vector3[] jointStartDirection { get; set; }
    private Rotation ikTargetStartRot { get; set; }
    private Rotation lastJointStartRot { get; set; }

	[Property] public GameObject poleTarget;

	[Property] public bool debugJoints = true;
	[Property] public bool localRotationAxis = false;

    // Remove this if you need bigger gizmos:
    [Property, Range(0.0f, 40f)]
    public float gizmoSize = 2f;

    public bool poleDirection = false;
    public bool poleRotationAxis = false;

	protected override void OnStart()
	{
		Awake();
	}
	protected override async void OnEnabled()
	{
		Awake();
	}
	void Awake()
    {
        // Let's set some properties
        jointChainLength = 0;
        jointTransforms = new GameObject[numberOfJoints];
        jointPositions = new Vector3[numberOfJoints];
        boneLength = new float[numberOfJoints];
        jointStartDirection = new Vector3[numberOfJoints];
        startRotation = new Rotation[numberOfJoints];
        ikTargetStartRot = ikTarget.WorldRotation;

        var current = GameObject;

        // For each bone calculate and store the lenght of the bone
        for (var i = 0; i < jointTransforms.Length; i++)
        {
            jointTransforms[i] = current;
            // If the bones lenght equals the max lenght, we are on the last joint in the chain
            if (i == jointTransforms.Length - 1)
            {
                lastJointStartRot = current.WorldRotation;
                return;
            }
            // Store length and add the sum of the bone lengths
            else
            {
                boneLength[i] = Vector3.DistanceBetween(current.WorldPosition, current.Children[0].WorldPosition );
                jointChainLength += boneLength[i];

                jointStartDirection[i] = current.Children[0].WorldPosition - current.WorldPosition;
                startRotation[i] = current.WorldRotation;
            }
            // Move the iteration to next joint in the chain
            current = current.Children[0];
        }
    }

    void PoleConstraint()
    {
        if (poleTarget != null && numberOfJoints < 4)
        {
            // Get the limb axis direction
            var limbAxis = (jointPositions[2] - jointPositions[0]).Normal;

            // Get the direction from the root joint to the pole target and mid joint
            Vector3 poleDirection = (poleTarget.WorldPosition - jointPositions[0]).Normal;
            Vector3 boneDirection = (jointPositions[1] - jointPositions[0]).Normal;
            
            // Ortho-normalize the vectors
            OrthoNormalize(ref limbAxis, ref poleDirection);
            OrthoNormalize(ref limbAxis, ref boneDirection);

            // Calculate the angle between the boneDirection vector and poleDirection vector
            Rotation angle = Rotation.FromToRotation(boneDirection, poleDirection);

            // Rotate the middle bone using the angle
            jointPositions[1] = angle * (jointPositions[1] - jointPositions[0]) + jointPositions[0];
        }
    }

	public static void OrthoNormalize( ref Vector3 normal, ref Vector3 tangent )
	{
		normal = normal.Normal;

		tangent -= Vector3.Dot( tangent, normal ) * normal;

		tangent = tangent.Normal;
	}

	void Backward()
    {
        // Iterate through every WorldPosition in the list until we reach the start of the chain
        for (int i = jointPositions.Length - 1; i >= 0; i -= 1)
        {
            // The last bone WorldPosition should have the same WorldPosition as the ikTarget
            if (i == jointPositions.Length - 1)
            {
                jointPositions[i] = ikTarget.WorldPosition;
            }
            else
            {
                jointPositions[i] = jointPositions[i + 1] + (jointPositions[i] - jointPositions[i + 1]).Normal * boneLength[i];
            }
        }
    }

    void Forward()
    {
        // Iterate through every WorldPosition in the list until we reach the end of the chain
        for (int i = 0; i < jointPositions.Length; i += 1)
        {
            // The first bone WorldPosition should have the same WorldPosition as the startPosition
            if (i == 0)
            {
                jointPositions[i] = startPosition;
            }
            else
            {
                jointPositions[i] = jointPositions[i - 1] + (jointPositions[i] - jointPositions[i - 1]).Normal * boneLength[i - 1];
            }
        }
    }

    private void SolveIK()
    {
        // Get the jointPositions from the joints
        for (int i = 0; i < jointTransforms.Length; i += 1)
        {
            jointPositions[i] = jointTransforms[i].WorldPosition;
        }
        // Distance from the root to the ikTarget
        distanceToTarget = Vector3.DistanceBetween(jointPositions[0], ikTarget.WorldPosition);

        // IF THE TARGET IS NOT REACHABLE
        if (distanceToTarget > jointChainLength)
        {
            // Direction from root to ikTarget
            var direction = ikTarget.WorldPosition - jointPositions[0];

            // Get the jointPositions
            for (int i = 1; i < jointPositions.Length; i += 1)
            {
                jointPositions[i] = jointPositions[i - 1] + direction.Normal * boneLength[i - 1];
            }
        }
        // IF THE TARGET IS REACHABLE
        else
        {
            // Get the distance from the leaf bone to the ikTarget
            float distToTarget = Vector3.DistanceBetween(jointPositions[jointPositions.Length - 1], ikTarget.WorldPosition);
            float counter = 0;
            // While the distance to target is greater than the tolerance let's iterate until we are close enough
            while (distToTarget > tolerance)
            {
                startPosition = jointPositions[0];
                Backward();
                Forward();
                counter += 1;
                // After x iterations break the loop to avoid an infinite loop
                if (counter > iterations)
                {
                    break;
                }
            }
        }
        // Apply the pole constraint
        PoleConstraint();

        // Apply the jointPositions and rotations to the joints
        for (int i = 0; i < jointPositions.Length - 1; i += 1)
        {
            jointTransforms[i].WorldPosition = jointPositions[i];
            var targetRotation = Rotation.FromToRotation(jointStartDirection[i], jointPositions[i + 1] - jointPositions[i]);
            jointTransforms[i].WorldRotation = targetRotation * startRotation[i];
        }
        // Let's constrain the rotation of the last joint to the IK target and maintain the offset.
        Rotation offset = lastJointStartRot * ikTargetStartRot.Inverse;
        jointTransforms.Last().WorldRotation = ikTarget.WorldRotation * offset;
    }

    protected override void OnUpdate()
    {
        SolveIK();
    }

    // Visual debugging
    protected override void DrawGizmos()
    {
        if (debugJoints == true)
        {   
            var current = GameObject;
            var child = GameObject.Children[0];
			Gizmo.Draw.Color = Color.Cyan;
            for (int i = 0; i < numberOfJoints; i += 1)
            {
                if (i == numberOfJoints - 1)
                {
					Gizmo.Draw.LineCapsule( new Capsule( WorldTransform.PointToLocal(current.WorldPosition), WorldTransform.PointToLocal( child.WorldPosition ), gizmoSize) );
			break;
                }
                else
                {
					Gizmo.Draw.LineCapsule( new Capsule(WorldTransform.PointToLocal( current.WorldPosition ), WorldTransform.PointToLocal( child.WorldPosition ), gizmoSize) );
					current = current.Children[0];
                    child = current.Children[0];
                }
            }
        }

        if (localRotationAxis == true)
        {    
            var current = GameObject;
            for (int i = 0; i < numberOfJoints; i += 1)
            {
                if (i == numberOfJoints - 1)
                {
                    drawHandle(current);
                }
                else
                {
                    drawHandle(current);
                    current = current.Children[0];
                }
            }
        }

        var start = GameObject;
		var mid = start.Children[0];
        var end = mid.Children[0];

		if (poleRotationAxis == true && poleTarget != null && numberOfJoints < 4)
        {    
            Gizmo.Draw.Color = Color.White;
            Gizmo.Draw.Line(start.WorldPosition, end.WorldPosition);
        }

        if (poleDirection == true && poleTarget != null && numberOfJoints < 4)
        {
			Gizmo.Draw.Color = Color.Gray;
			Gizmo.Draw.Line( start.WorldPosition, poleTarget.WorldPosition);
			Gizmo.Draw.Line( end.WorldPosition, poleTarget.WorldPosition);
        }

		if(ikTarget.IsValid())
		{
			Gizmo.Draw.Color = Color.Green;
			Gizmo.Draw.SolidSphere( WorldTransform.PointToLocal(ikTarget.WorldPosition), gizmoSize );
		}

		if(poleTarget.IsValid())
		{
			Gizmo.Draw.Color = Color.Red;
			Gizmo.Draw.SolidSphere( WorldTransform.PointToLocal(  poleTarget.WorldPosition ), gizmoSize );
		}
	}

    void drawHandle(GameObject debugJoint)
    {
        Gizmo.Draw.Color = Color.Red;
		Gizmo.Draw.Arrow( WorldTransform.PointToLocal( debugJoint.WorldPosition ), WorldTransform.PointToLocal( debugJoint.WorldPosition + Vector3.Forward * gizmoSize ), gizmoSize * 0.1f, gizmoSize * 0.1f );

		Gizmo.Draw.Color = Color.Yellow;
		Gizmo.Draw.Arrow( WorldTransform.PointToLocal( debugJoint.WorldPosition ), WorldTransform.PointToLocal( debugJoint.WorldPosition + Vector3.Right * gizmoSize ), gizmoSize * 0.1f, gizmoSize * 0.1f );

		Gizmo.Draw.Color = Color.Blue;
		Gizmo.Draw.Arrow( WorldTransform.PointToLocal( debugJoint.WorldPosition ), WorldTransform.PointToLocal( debugJoint.WorldPosition + Vector3.Up * gizmoSize ), gizmoSize * 0.1f, gizmoSize * 0.1f );
	}
}
