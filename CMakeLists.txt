project(CFE_MPU6050 C)

# Create the app module
add_cfe_app(mpu6050 fsw/src/mpu6050.c)

# The API to this library (which may be invoked/referenced from other apps)
# is stored in fsw/public_inc.
target_include_directories(mpu6050 PUBLIC fsw/public_inc)
