# Trabajo de Final de Grado - Robot LIMO con Conducción Autónoma y Detección de Señales

Este repositorio contiene el proyecto del Trabajo de Final de Grado (TFG) realizado en el marco de la **Grado en Ingeniería Informática** de la **Universidad de Barcelona (UB)**. El proyecto se centra en el campo de la **robótica**, y en concreto, en el estudio, configuración y uso del robot **LIMO** (de AgileX Robotics) para tareas de **conducción autónoma** y **detección de señales de tráfico**.

## Objetivo

El objetivo principal del TFG es desarrollar un sistema que permita al robot LIMO navegar de forma autónoma en un entorno simulado o real, utilizando técnicas de visión por computador para detectar y reconocer señales de tráfico, y adaptar su comportamiento de navegación en consecuencia.

## Características principales

- Configuración del entorno de desarrollo con **ROS2** (y compatibilidad con **ROS Noetic**).
- Simulación del entorno de navegación utilizando herramientas como **Gazebo**, **RViz** o entornos personalizados.
- Control de movimiento autónomo mediante navegación planificada.
- Detección y reconocimiento de señales mediante técnicas de **machine learning / deep learning** aplicadas a imágenes.
- Integración de los módulos de percepción y control para una experiencia completa de conducción autónoma.

## Estructura del proyecto

- `src/`: Código fuente del sistema.
- `scripts/`: Scripts de utilidad para lanzar nodos o configurar el entorno.
- `models/`: Modelos entrenados para detección de señales.
- `launch/`: Archivos de lanzamiento para ROS.
- `docs/`: Documentación técnica del proyecto.

## Requisitos

- Ubuntu 20.04 o superior
- ROS Noetic
- ROS2 Foxy o Humble (dependiendo de la configuración)
- Python 3.8+
- OpenCV
- TensorFlow (modelo de detección)
- Keras
- Gazebo (para simulación)
- Dependencias de AgileX Robotics y LIMO SDK

## Instalación

1. Clona el repositorio:
   ```bash
   git clone https://github.com/Mattyete/TFG_2025.git
   cd TFG_2025
