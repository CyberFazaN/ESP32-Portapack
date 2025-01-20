
#include "orientation.h"
#include "nvs_flash.h"
#include <string.h>
#include "sensordb.h"

float declinationAngle = 0; //  setup to web interface https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml

hmc5883l_dev_t dev_hmc5883l;
i2c_dev_t dev_adxl345;
i2c_dev_t dev_mpu925x;
i2c_dev_t dev_ak8963;
qmc5883l_t dev_qmc5883l;
lsm303_t dev_lsm303;

OrientationSensors orientation_inited = Orientation_none;
AcceloSensors accelo_inited = Accelo_none;
bool ak8963_inited = false;

float accelo_x = 0;
float accelo_y = 0;
float accelo_z = 0;

float m_tilt = 400.0;

int16_t orientationXMin = INT16_MAX;
int16_t orientationYMin = INT16_MAX;
int16_t orientationZMin = INT16_MAX;
int16_t orientationXMax = INT16_MIN;
int16_t orientationYMax = INT16_MIN;
int16_t orientationZMax = INT16_MIN;


calibration_t cal = {  // temporary for testing
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
    .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
    .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
    .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},
    .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782}
};


void warn_not_init(char *tag, char *name, uint8_t addr) {
    ESP_LOGW(tag, "%s probably found at 0x%02x, but not initialized", name, addr);
}

void init_accelo()
{
    memset(&dev_adxl345, 0, sizeof(dev_adxl345));
    uint8_t adxl345_addr = getDevAddr(ADXL345);
    if (adxl345_addr != 0 && adxl345_init_desc(&dev_adxl345, adxl345_addr, 0, CONFIG_IC2SDAPIN, CONFIG_IC2SCLPIN) == ESP_OK && adxl345_init(&dev_adxl345) == ESP_OK)
    {
        accelo_inited |= Accelo_ADXL345;
        ESP_LOGI("Acceleration", "ADXL345 OK");
    }
    else
    {
        if (adxl345_addr != 0)
            warn_not_init("Acceleration", "ADXL345", adxl345_addr);
        adxl345_free_desc(&dev_adxl345);
    }
}

void init_orientation()
{
    // load calibration data
    load_config_orientation();
    accelo_inited = Accelo_none;
    orientation_inited = Orientation_none;

    // HCM5883l
    uint8_t hmc5883l_addr = getDevAddr(HMC5883L);
    if (hmc5883l_addr != 0){
        memset(&dev_hmc5883l, 0, sizeof(hmc5883l_dev_t));
        if (hmc5883l_init_desc(&dev_hmc5883l, 0, CONFIG_IC2SDAPIN, CONFIG_IC2SCLPIN, hmc5883l_addr) == ESP_OK && hmc5883l_init(&dev_hmc5883l) == ESP_OK)
        {
            hmc5883l_set_opmode(&dev_hmc5883l, HMC5883L_MODE_CONTINUOUS);
            hmc5883l_set_samples_averaged(&dev_hmc5883l, HMC5883L_SAMPLES_8);
            hmc5883l_set_data_rate(&dev_hmc5883l, HMC5883L_DATA_RATE_07_50);
            hmc5883l_set_gain(&dev_hmc5883l, HMC5883L_GAIN_1090);
            ESP_LOGI("Orientation", "HMC5883L TestConnection OK");
            orientation_inited |= Orientation_hmc5883l;
        }
        else
        {
            if (hmc5883l_addr != 0)
                warn_not_init("Orientation", "HMC5883L", hmc5883l_addr);
            hmc5883l_free_desc(&dev_hmc5883l);
        }
    }

    // QCM5883l
    uint8_t qmc5883l_addr = getDevAddr(QMC5883L);
    if (qmc5883l_addr != 0){
        memset(&dev_qmc5883l, 0, sizeof(qmc5883l_t));
        if (qmc5883l_init_desc(&dev_qmc5883l, qmc5883l_addr, 0, CONFIG_IC2SDAPIN, CONFIG_IC2SCLPIN) == ESP_OK)
        {
            qmc5883l_set_mode(&dev_qmc5883l, QMC5883L_MODE_CONTINUOUS);
            // qmc5883l_set_samples_averaged(&dev_qmc5883l, QMC5883L_SAMPLES_8);
            // qmc5883l_set_data_rate(&dev_qmc5883l, QMC5883L_DATA_RATE_07_50);
            // qmc5883l_set_gain(&dev_qmc5883l, QMC5883L_GAIN_1090);
            ESP_LOGI("Orientation", "QMC5883L OK");
            orientation_inited |= Orientation_qmc5883l;
        }
        else
        {
            if (qmc5883l_addr != 0)
                warn_not_init("Orientation", "QMC5883L", qmc5883l_addr);
            hmc5883l_free_desc(&dev_hmc5883l);
        }
    }

    // MPU9250
    uint8_t mpu925x_addr = getDevAddr(MPU925X);
    uint8_t ak8963_addr = getDevAddr(AK8963);
    if (mpu925x_addr != 0 && ak8963_addr != 0){
        memset(&dev_mpu925x, 0, sizeof(dev_mpu925x));
        memset(&dev_ak8963, 0, sizeof(dev_ak8963));
        if (ak8963_init_desc(&dev_ak8963, ak8963_addr, 0, CONFIG_IC2SDAPIN, CONFIG_IC2SCLPIN) == ESP_OK){
            if (mpu9250_init_desc(&dev_mpu925x, mpu925x_addr, 0, CONFIG_IC2SDAPIN, CONFIG_IC2SCLPIN) == ESP_OK && mpu9250_init_with_mag(&dev_mpu925x, &dev_ak8963, &cal) == ESP_OK){
                accelo_inited |= Accelo_MPU925x; // todo check if really available
                orientation_inited |= Orientation_mpu925x;
                ak8963_inited = true;
                ESP_LOGI("Orientation", "MPU925x + AK8963 OK");
            }else{
                mpu9250_free_desc(&dev_mpu925x);
                ak8963_free_desc(&dev_ak8963);
                warn_not_init("Orientation", "MPU925x", mpu925x_addr);
                warn_not_init("Orientation", "AK8963", ak8963_addr);
            }
        }
    }else if (mpu925x_addr != 0){
        memset(&dev_mpu925x, 0, sizeof(dev_mpu925x));
        if (mpu9250_init_desc(&dev_mpu925x, mpu925x_addr, 0, CONFIG_IC2SDAPIN, CONFIG_IC2SCLPIN) == ESP_OK && mpu9250_init(&dev_mpu925x, &cal) == ESP_OK){
                accelo_inited |= Accelo_MPU925x; // todo check if really available
                orientation_inited |= Orientation_mpu925x;
                ESP_LOGW("Orientation", "MPU925x OK, but no AK8963 (mag)");
        }else{
            mpu9250_free_desc(&dev_mpu925x);
            warn_not_init("Orientation", "MPU925x", mpu925x_addr);
        }
    }

    uint8_t lsm303_accel_addr = getDevAddr(LSM303_ACCEL);
    uint8_t lsm303_mag_addr = getDevAddr(LSM303_MAG);
    if (lsm303_accel_addr != 0 && lsm303_mag_addr != 0 ){
        memset(&dev_lsm303, 0, sizeof(dev_lsm303));
        if (lsm303_init_desc(&dev_lsm303, lsm303_accel_addr, lsm303_mag_addr, 0, CONFIG_IC2SDAPIN, CONFIG_IC2SCLPIN) == ESP_OK && lsm303_init(&dev_lsm303) == ESP_OK)
        {
            accelo_inited |= Accelo_LSM303;
            orientation_inited |= Orientation_lsm303;
            lsm303_acc_set_config(&dev_lsm303, LSM303_ACC_MODE_NORMAL, LSM303_ODR_100_HZ, LSM303_ACC_SCALE_2G);
            lsm303_mag_set_config(&dev_lsm303, LSM303_MAG_MODE_CONT, LSM303_MAG_RATE_15, LSM303_MAG_GAIN_1_3);
            ESP_LOGI("Orientation", "LSM303 accel+mag OK");
        }
        else
        {
            if (lsm303_accel_addr != 0)
                warn_not_init("Orientation", "LSM303_accel", lsm303_accel_addr);
            if (lsm303_mag_addr != 0)
                warn_not_init("Orientation", "LSM303_mag", lsm303_mag_addr);
            lsm303_free_desc(&dev_lsm303);
        }
    }

    // end of list
    if (orientation_inited == Orientation_none)
    {
        ESP_LOGW("Orientation", "No compatible sensor found");
        // return;
    }

    init_accelo();
}

float fix_heading(float heading)
{
    if (heading < 0)
        heading += 2 * M_PI;

    if (heading > 2 * M_PI)
        heading -= 2 * M_PI;
    return (heading + declinationAngle);
}

void update_gyro_data()
{
    if (accelo_inited == Accelo_none)
        return;
    if ((accelo_inited & Accelo_ADXL345) == Accelo_ADXL345)
    {
        adxl345_read_x(&dev_adxl345, &accelo_x);
        adxl345_read_y(&dev_adxl345, &accelo_y);
        adxl345_read_z(&dev_adxl345, &accelo_z);
        ESP_LOGD("Accelo_ADXL345", "Accel data: %f %f %f", accelo_x, accelo_y, accelo_z);
    }

    if ((accelo_inited & Accelo_MPU925x) == Accelo_MPU925x) {
        vector_t accel_vector;
        esp_err_t ret = mpu9250_get_accel(&dev_mpu925x, &accel_vector);
        if (ret == ESP_OK) {
            accelo_x = accel_vector.x;
            accelo_y = accel_vector.y;
            accelo_z = accel_vector.z;

            ESP_LOGD("Accelo_MPU925x", "Accel data: X=%f Y=%f Z=%f", accelo_x, accelo_y, accelo_z);
        } else {
            ESP_LOGW("Accelo_MPU925x", "Failed to read accelerometer data: %s", esp_err_to_name(ret));
        }
    }

    if ((accelo_inited & Accelo_LSM303) == Accelo_LSM303)
    {
        lsm303_acc_raw_data_t acc_raw;
        lsm303_acc_data_t acc;

        if (lsm303_acc_get_raw_data(&dev_lsm303, &acc_raw) == ESP_OK)
        {
            lsm303_acc_raw_to_g(&dev_lsm303, &acc_raw, &acc);
            accelo_x = acc.x;
            accelo_y = acc.y;
            accelo_z = acc.z;
            ESP_LOGD("Accelo_LSM303", "Accel data: %f %f %f", accelo_x, accelo_y, accelo_z);
        }
    }
}

float get_heading()
{
    if (orientation_inited == Orientation_none)
        return 0;
    float ret = 0.0;
    update_gyro_data(); // update only if orientation needs it.
    if ((orientation_inited & Orientation_hmc5883l) == Orientation_hmc5883l)
    {
        hmc5883l_data_t data;
        if (hmc5883l_get_data(&dev_hmc5883l, &data) == ESP_OK)
        {
            if (accelo_inited == Accelo_none || (accelo_x == 0 && accelo_y == 0))
                ret = atan2(data.y, data.x);
            else
            {
                // if got gyro data
                float accXnorm = accelo_x / sqrt(accelo_x * accelo_x + accelo_y * accelo_y + accelo_z * accelo_z);
                float accYnorm = accelo_y / sqrt(accelo_x * accelo_x + accelo_y * accelo_y + accelo_z * accelo_z);
                float pitch = asin(-accXnorm);
                float roll = asin(accYnorm / cos(pitch));
                m_tilt = roll * 2 * M_PI;
                // todo use calibration
                float magXcomp = data.x; // (magX - calibration[0]) / (calibration[1] - calibration[0]) * 2 - 1;
                float magYcomp = data.y; //(magY - calibration[2]) / (calibration[3] - calibration[2]) * 2 - 1;
                float magZcomp = data.z; //(magZ - calibration[4]) / (calibration[5] - calibration[4]) * 2 - 1;

                float magXheading = magXcomp * cos(pitch) + magZcomp * sin(pitch);
                float magYheading = magXcomp * sin(roll) * sin(pitch) + magYcomp * cos(roll) - magZcomp * sin(roll) * cos(pitch);

                ret = atan2(magYheading, magXheading);
            }
        }else{
            ESP_LOGW("Orientation_HMC5883L", "Failed to get mag data");
        }
    }
    if ((orientation_inited & Orientation_qmc5883l) == Orientation_qmc5883l)
    {
        qmc5883l_data_t data;
        if (qmc5883l_get_data(&dev_qmc5883l, &data) == ESP_OK)
        {
            if (accelo_inited == Accelo_none || (accelo_x == 0 && accelo_y == 0))
                ret = atan2(data.y, data.x);
            else
            {
                // if got gyro data
                float accXnorm = accelo_x / sqrt(accelo_x * accelo_x + accelo_y * accelo_y + accelo_z * accelo_z);
                float accYnorm = accelo_y / sqrt(accelo_x * accelo_x + accelo_y * accelo_y + accelo_z * accelo_z);
                float pitch = asin(-accXnorm);
                float roll = asin(accYnorm / cos(pitch));
                m_tilt = roll * 2 * M_PI;
                // todo use calibration
                float magXcomp = data.x; // (magX - calibration[0]) / (calibration[1] - calibration[0]) * 2 - 1;
                float magYcomp = data.y; //(magY - calibration[2]) / (calibration[3] - calibration[2]) * 2 - 1;
                float magZcomp = data.z; //(magZ - calibration[4]) / (calibration[5] - calibration[4]) * 2 - 1;

                float magXheading = magXcomp * cos(pitch) + magZcomp * sin(pitch);
                float magYheading = magXcomp * sin(roll) * sin(pitch) + magYcomp * cos(roll) - magZcomp * sin(roll) * cos(pitch);

                ret = atan2(magYheading, magXheading);
            }
        }else{
            ESP_LOGW("Orientation_QMC5883L", "Failed to get mag data");
        }
    }
    if ((orientation_inited & Orientation_mpu925x) == Orientation_mpu925x && ak8963_inited)
    {
        int16_t magX = 0;
        int16_t magY = 0;
        int16_t magZ = 0;
        vector_t mag_vector;
        if (mpu9250_get_mag(&dev_mpu925x, &mag_vector) == ESP_OK)
        {
            magX = mag_vector.x;
            magY = mag_vector.y;
            magZ = mag_vector.z;
            ESP_LOGD("Orientation_MPU925x", "mxyz: %d %d %d", magX, magY, magZ);
            if (accelo_inited == Accelo_none || (accelo_x == 0 && accelo_y == 0))
                ret = atan2(magY, magX);
            else
            {
                // if got gyro data
                float accXnorm = accelo_x / sqrt(accelo_x * accelo_x + accelo_y * accelo_y + accelo_z * accelo_z);
                float accYnorm = accelo_y / sqrt(accelo_x * accelo_x + accelo_y * accelo_y + accelo_z * accelo_z);
                float pitch = asin(-accXnorm);
                float roll = asin(accYnorm / cos(pitch));
                m_tilt = roll * 2 * M_PI;
                // todo use calibration
                float magXcomp = magX; // (magX - calibration[0]) / (calibration[1] - calibration[0]) * 2 - 1;
                float magYcomp = magY; //(magY - calibration[2]) / (calibration[3] - calibration[2]) * 2 - 1;
                float magZcomp = magZ; //(magZ - calibration[4]) / (calibration[5] - calibration[4]) * 2 - 1;

                float magXheading = magXcomp * cos(pitch) + magZcomp * sin(pitch);
                float magYheading = magXcomp * sin(roll) * sin(pitch) + magYcomp * cos(roll) - magZcomp * sin(roll) * cos(pitch);

                ret = atan2(magYheading, magXheading);
            }
        }
        else
        {
            ESP_LOGW("Orientation_MPU925x", "Failed to get mag data");
        }
    }

    if ((orientation_inited & Orientation_lsm303) == Orientation_lsm303)
    {
        lsm303_mag_raw_data_t mag_raw;
        lsm303_mag_data_t mag;

        int16_t magX = 0;
        int16_t magY = 0;
        int16_t magZ = 0;
        if (lsm303_mag_get_raw_data(&dev_lsm303, &mag_raw) == ESP_OK)
        {
            lsm303_mag_raw_to_uT(&dev_lsm303, &mag_raw, &mag);
            magX = mag.x;
            magY = mag.y;
            magZ = mag.z;

            ESP_LOGD("Orientation_LSM303", "mxyz: %d %d %d", magX, magY, magZ);
            if (accelo_inited == Accelo_none || (accelo_x == 0 && accelo_y == 0))
                ret = atan2(magY, magX);
            else
            {
                // if got gyro data
                float accXnorm = accelo_x / sqrt(accelo_x * accelo_x + accelo_y * accelo_y + accelo_z * accelo_z);
                float accYnorm = accelo_y / sqrt(accelo_x * accelo_x + accelo_y * accelo_y + accelo_z * accelo_z);
                float pitch = asin(-accXnorm);
                float roll = asin(accYnorm / cos(pitch));
                m_tilt = roll * 2 * M_PI;
                // todo use calibration
                float magXcomp = magX; // (magX - calibration[0]) / (calibration[1] - calibration[0]) * 2 - 1;
                float magYcomp = magY; //(magY - calibration[2]) / (calibration[3] - calibration[2]) * 2 - 1;
                float magZcomp = magZ; //(magZ - calibration[4]) / (calibration[5] - calibration[4]) * 2 - 1;

                float magXheading = magXcomp * cos(pitch) + magZcomp * sin(pitch);
                float magYheading = magXcomp * sin(roll) * sin(pitch) + magYcomp * cos(roll) - magZcomp * sin(roll) * cos(pitch);

                ret = atan2(magYheading, magXheading);
            }
        }
        else
        {
            ESP_LOGW("Orientation_LSM303", "Failed to get mag data");
        }
    }
    ESP_LOGD("Orientation", "%f", ret);
    return ret;
}

float get_heading_degrees()
{
    if (orientation_inited == Orientation_none)
        return 400;
    float heading = get_heading() * 180 / M_PI + declinationAngle;
    if (heading < 0)
        heading += 360;
    ESP_LOGD("Heading", "%f", heading);
    return heading;
}

void calibrate_orientation(uint8_t sec)
{
    // todo, run a calibration for N sec to get min-max. should be around 10

    save_config_orientation();
}

void set_declination(float declination)
{
    declinationAngle = declination;
    save_config_orientation();
}
float get_declination() { return declinationAngle; }

float get_tilt() { return m_tilt; }

bool is_orientation_sensor_present()
{
    return orientation_inited != Orientation_none;
}