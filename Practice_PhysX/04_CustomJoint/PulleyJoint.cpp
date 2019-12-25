#include "PulleyJoint.h"


using namespace physx;

// ShaderTable에 사용될 함수들 등록.
PxConstraintShaderTable PulleyJoint::m_ShaderTable = { 
	&PulleyJoint::SolverPrep, // 물리 처리 이전에 필요한 작업들
	&PulleyJoint::Project,
	&PulleyJoint::Visualize, // PVD 에서 확인할 수 있도록 그래픽 작업.
	PxConstraintFlag::Enum(0) };

PulleyJoint::PulleyJoint(PxPhysics& physics, 
	PxRigidBody& body0, const PxTransform& localFrame0, const PxVec3& attachment0, 
	PxRigidBody& body1, const PxTransform& localFrame1, const PxVec3& attachment1)
{
	m_Constraint = physics.createConstraint(
		&body0, &body1, 
		*this, 
		m_ShaderTable,			// 주요 동작 함수들의 테이블 등록. 
		sizeof(PulleyJointData) // prepareData() 로 받는 데이터의 사이즈.
	);

	m_Body[0] = &body0;
	m_Body[1] = &body1;

	m_LocalOffset[0] = localFrame0.getNormalized();
	m_LocalOffset[1] = localFrame1.getNormalized();

	m_Data.attachment0 = attachment0;
	m_Data.attachment1 = attachment1;
	m_Data.distance = 1.0f;
	m_Data.ratio = 1.0f;
	m_Data.localJointPoint[0] = body0.getCMassLocalPose().transformInv(m_LocalOffset[0]);
	m_Data.localJointPoint[1] = body1.getCMassLocalPose().transformInv(m_LocalOffset[1]);
}

void PulleyJoint::Release()
{
	m_Constraint->release();
}

void PulleyJoint::SetAttachment0(const PxVec3& pos)
{
	m_Data.attachment0 = pos;
	m_Constraint->markDirty();
}

PxVec3 PulleyJoint::GetAttachment0() const
{
	return m_Data.attachment0;
}

void PulleyJoint::SetAttachment1(const PxVec3& pos)
{
	m_Data.attachment1 = pos;
	m_Constraint->markDirty();
}

PxVec3 PulleyJoint::GetAttachment1() const
{
	return m_Data.attachment1;
}

void PulleyJoint::SetDistance(PxReal totalDistance)
{
	m_Data.distance = totalDistance;
	m_Constraint->markDirty();
}

PxReal PulleyJoint::GetDistance() const
{
	return m_Data.distance;
}

void PulleyJoint::SetRatio(PxReal ratio)
{
	m_Data.ratio = ratio;
	m_Constraint->markDirty();
}

PxReal PulleyJoint::GetRatio() const
{
	return m_Data.ratio;
}

//////////////////////////////////////////

void* PulleyJoint::prepareData()
{
	return &m_Data;
}

void PulleyJoint::onConstraintRelease()
{
	delete this;
}

void PulleyJoint::onComShift(PxU32 actor)
{
	m_Data.localJointPoint[actor] = m_Body[actor]->getCMassLocalPose().transformInv(m_LocalOffset[actor]);
	m_Constraint->markDirty();
}

void PulleyJoint::onOriginShift(const PxVec3& shift)
{
	m_Data.attachment0 -= shift;
	m_Data.attachment1 -= shift;
}

void* PulleyJoint::getExternalReference(PxU32& typeID)
{
	typeID = TYPE_ID;
	return this;
}


/////////////////////////////////

PxU32 PulleyJoint::SolverPrep(
	Px1DConstraint* constraints, 
	PxVec3& body0WorldOffset,
	PxU32 maxConstraints, 
	PxConstraintInvMassScale&, 
	const void* constantBlock, 
	const PxTransform& body0World,
	const PxTransform& body1World,
	bool useExtendedLimits, 
	PxVec3& body0WorldOut, 
	PxVec3& body1WorldOut)
{
	const PulleyJointData& data = *reinterpret_cast<const PulleyJointData*>(constantBlock);

	// 각 바디의 jointPoint의 월드상 위치
	PxTransform body0Joint = body0World.transform(data.localJointPoint[0]);
	PxTransform body1Joint = body1World.transform(data.localJointPoint[1]);

	body0WorldOut = body0Joint.p;
	body1WorldOut = body1Joint.p;

	body0WorldOffset = body1Joint.p - body0World.p;

	// 바디가 연결된 어태치와의 거리를 각각 계산.
	PxVec3 vector_Body0ToAttach0 = data.attachment0 - body0Joint.p;
	PxReal distanceVec0 = vector_Body0ToAttach0.normalize();

	PxVec3 vector_Body1ToAttach1 = data.attachment1 - body1Joint.p;
	PxReal distanceVec1 = vector_Body1ToAttach1.normalize();

	distanceVec1 *= data.ratio;

	PxReal totalDistance = distanceVec0 + distanceVec1;

	PxReal geometricError = (data.distance - totalDistance);

	//https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxapi/files/structPx1DConstraint.html
	Px1DConstraint* c = constraints;
	c->flags = Px1DConstraintFlag::eOUTPUT_FORCE;

	if (geometricError < 0.0f)
	{
		c->maxImpulse = PX_MAX_F32;
		c->minImpulse = 0;
		c->geometricError = geometricError;
	}
	else if (geometricError > 0.0f)
	{
		c->maxImpulse = 0;
		c->minImpulse = -PX_MAX_F32;
		c->geometricError = geometricError;
	}

	c->linear0 = vector_Body0ToAttach0;
	c->angular0 = (body0Joint.p - body0World.p).cross(c->linear0);

	c->linear1 = -vector_Body1ToAttach1;
	c->angular1 = (body1Joint.p - body1World.p).cross(c->linear1);

	return 1;
}

void PulleyJoint::Visualize(PxConstraintVisualizer& viz, 
	const void* constantBlock, 
	const PxTransform& body0Transform, 
	const PxTransform& body1Transform, PxU32 flags)
{
	const PulleyJointData& data = *reinterpret_cast<const PulleyJointData*>(constantBlock);

	PxTransform body0Joint = body0Transform.transform(data.localJointPoint[0]);
	PxTransform body1Joint = body1Transform.transform(data.localJointPoint[1]);

	viz.visualizeLine(body0Joint.p, data.attachment0, 0xff000000);
	viz.visualizeLine(body1Joint.p, data.attachment1, 0xff000000);
}

void PulleyJoint::Project(const void* constantBlock, PxTransform& bodyAToWorld, PxTransform& bodyBToWorld, bool projectToA)
{
}
