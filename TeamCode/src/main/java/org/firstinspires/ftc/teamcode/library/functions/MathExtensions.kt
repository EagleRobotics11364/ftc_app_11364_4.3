package org.firstinspires.ftc.teamcode.library.functions

fun Double.toDegrees() = this * 180 / Math.PI
fun Double.toRadians() = this * Math.PI / 180
fun Double.truncate(decimals: Byte) = String.format("%.${decimals}f", this).toDouble()