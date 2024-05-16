/**
 * @file chi_squared_tables.h
 * @author Sven Pfeiffer, MAVLab, TU Delft
 * @date 01 May 2024
 * @brief Tail probabilities for chi squared distribution
 * 
 * This header contains selected tail probabilities for the chi
 * squared distribution, to be used for testing the normalized inovation
 * squared (NIS) or Normalized Estimation Error Squared (NEES)
 */ 

#ifndef _CHI_SQUARED_TABLES_H_
#define _CHI_SQUARED_TABLES_H_

// 1 DOF
#define CHI_SQUARED_1_0999   10.80f
#define CHI_SQUARED_1_0995    7.88f
#define CHI_SQUARED_1_0990    6.63f
#define CHI_SQUARED_1_0975    5.02f
#define CHI_SQUARED_1_0950    3.84f
#define CHI_SQUARED_1_0900    2.71f
#define CHI_SQUARED_1_0750    1.32f


// 20 DOF
#define CHI_SQUARED_20_0999   45.3f
#define CHI_SQUARED_20_0995   40.0f
#define CHI_SQUARED_20_0990   37.6f
#define CHI_SQUARED_20_0975   34.2f
#define CHI_SQUARED_20_0950   31.4f
#define CHI_SQUARED_20_0900   28.4f
#define CHI_SQUARED_20_0750   23.8f



#endif //_CHI_SQUARED_TABLES_H_