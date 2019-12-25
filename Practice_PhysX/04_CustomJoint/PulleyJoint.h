#pragma once

#include <PxPhysicsAPI.h>


/*
	 attachment0	attachment1
	   (*)			   (*)
		|				|
		|				|
		|				|
		|				|
	   [A]				|
	  body0				|
					   [B]
					  body1

	 vec3(body0 - attachment0) + (vec3(body1 - attachment1) * ratio) = distance

	 각 오브젝트를 개별적으로 특정 위치에 고정하고
	 정해진 총합 길이(distance로 표현)로 제한, 밀도차이로 움직이는 도르레를 만든다.
*/



class PulleyJoint :public physx::PxConstraintConnector
{
public:
	static const physx::PxU32 TYPE_ID = physx::PxConcreteType::eFIRST_USER_EXTENSION;

	PulleyJoint(physx::PxPhysics& physics,
		physx::PxRigidBody& body0, const physx::PxTransform& localFrame0, const physx::PxVec3& attachment0,
		physx::PxRigidBody& body1, const physx::PxTransform& localFrame1, const physx::PxVec3& attachment1);

	void Release() ;

	void			SetAttachment0(const physx::PxVec3& pos);
	physx::PxVec3	GetAttachment0() const;

	void			SetAttachment1(const physx::PxVec3& pos);
	physx::PxVec3	GetAttachment1() const;

	void			SetDistance(physx::PxReal totalDistance);
	physx::PxReal	GetDistance() const;

	void			SetRatio(physx::PxReal ratio);
	physx::PxReal	GetRatio() const;

private: //PxConstraintConnector 함수
	virtual void*	prepareData() override;
	virtual void	onConstraintRelease()override;
	virtual void	onComShift(physx::PxU32 actor)override;
	virtual void	onOriginShift(const physx::PxVec3& shift)override;
	virtual void*	getExternalReference(physx::PxU32& typeID)override;

	virtual bool	updatePvdProperties(physx::pvdsdk::PvdDataStream&,
					const physx::PxConstraint*,
					physx::PxPvdUpdateType::Enum) const override {return true;}

	physx::PxBase* getSerializable() override { return NULL; }
	virtual physx::PxConstraintSolverPrep getPrep() const override { return m_ShaderTable.solverPrep; }
	virtual const void* getConstantBlock() const override { return &m_Data; }

private: //ConstraintShaderTable에 등록될 함수들이다.
	static physx::PxU32 SolverPrep(physx::Px1DConstraint* constraints,
		physx::PxVec3& body0WorldOffset,
		physx::PxU32 maxConstraints,
		physx::PxConstraintInvMassScale&,
		const void* constantBlock,
		const physx::PxTransform& body0World,
		const physx::PxTransform& body1World,
		bool useExtendedLimits,
		physx::PxVec3& body0WorldOut, physx::PxVec3& body1WorldOut);


	static void	Visualize(physx::PxConstraintVisualizer& viz,
		const void* constantBlock,
		const physx::PxTransform& body0Transform,
		const physx::PxTransform& body1Transform,
		physx::PxU32 flags);

	static void Project(const void* constantBlock,
		physx::PxTransform& bodyAToWorld,
		physx::PxTransform& bodyBToWorld,
		bool projectToA);

private:
	struct PulleyJointData
	{
		physx::PxTransform localJointPoint[2];

		physx::PxVec3 attachment0;
		physx::PxVec3 attachment1;

		physx::PxReal distance;
		physx::PxReal ratio;

	} m_Data;

	physx::PxRigidBody*		m_Body[2];
	physx::PxTransform		m_LocalOffset[2];
	physx::PxConstraint*	m_Constraint;

	static physx::PxConstraintShaderTable m_ShaderTable;

	virtual ~PulleyJoint() {}
};