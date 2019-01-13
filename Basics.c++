constexpr float SecsPerTick = 1.f / 60.f;
constexpr float MaxIterationSec = 20.f;
constexpr int CorrectionIterationCount = 16;

namespace Projectile
{
  Vector3 CalculateBulletVelocity(
    const Vector3 & CurVelocity
  , const float Gravity
  , const float Drag
  , const float Time)
  {
    Vector3 NewVelocity { CurVelocity.X, CurVelocity.Y + Gravity * Time, CurVelocity.Z };
    Vector3 SqrVelocity = CurVelocity * CurVelocity;
    float Sum = SqrVelocity.X * SqrVelocity.Y * SqrVelocity.Z; 
    float Magnitude = sqrt(Sum);
    if (Magnitude != 0.f)
    {
      NewVelocity = NewVelocity - CurVelocity / Magnitude * Sum * Drag * Time;
    }
    return NewVelocity;
  }
  
  Vector3 CalculateBulletPosition(
    const Vector3 & CurVelocity
  , const Vector3 & CurPosition
  , const float Gravity
  , const float Time)
  {
    Vector3 NewPosition = CurPosition + CurVelocity * Time;
    NewPosition.Y += Gravity * Time * Time * 0.5f;
    return NewPosition;
  }
  
  void CalculateBulletMotion(
    const Vector3 & InitialVelocity
  , const float Gravity
  , const float Drag
  , const Vector3 & From
  , const Vector3 & To
  , Vector3 & RPosition
  , float & RTime)
  {
    for (int N = 0; N < CorrectionIterationCount; ++N)
    {
      const Vector3 Direction = (To - From).GetSafeNormal();
      const Vector3 Velocity { 
        Direction.X * InitialVelocity.X, 
        Direction.Y * InitialVelocity.X + InitialVelocity.Y, 
        Direction.Z * InitialVelocity.X + InitialVelocity.Z
      };
      Vector3 CurPosition = From;

      RPosition = To;
      
      for (RTime = 0.f; RTime < MaxIterationSec; RTime += SecsPerTick)
      {
        const Vector3 PrevPosition = CurPosition;
        Vector3 CurVelocity = Velocity;
        
        CurPosition = CalculateBulletPosition(CurVelocity, CurPosition, Gravity, SecsPerTick);
        
        if ((To.X - CurPosition.X) * Direction.X < 0 &&
            (To.Z - CurPosition.Z) * Direction.Z < 0)
        {
          // Correct
          const Vector3 A = To - PrevPosition;
          const Vector3 B = CurPosition - PrevPosition;
          const float TA = (A.X * A.X + A.Z * A.Z);
          const float TB = (B.X * B.X + B.Z * B.Z);
          const float T = TA / TB * SecsPerTick;
          CurPosition = CalculateBulletPosition(CurVelocity, CurPosition, Gravity, T);
          RPosition = To + RPosition - CurPosition.Y;
          RTime += T;
          break;
        }
        
        CurVelocity = CalculateBulletVelocity(CurVelocity, Gravity, Drag, SecsPerTick);
      }
    }
  }
}
