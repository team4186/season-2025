package frc.robot.parts

import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkLowLevel


class SimpleMotor (

)
{
    private val a_motor: SparkMax =  SparkMax(sparkmaxId, SparkLowLevel.MotorType.kBrushless)

    init {
        val config = MAXSwerveConfig()
        a_motor.configure(config., ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters)
    }

    fun drive(x: Double){

    }
}


//**
// Create Spark max object
// Controller, X -> send input to spark max
//
//
//
//
//
// */