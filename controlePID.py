# -*- coding: utf-8 -*-
'''
Criado por Vitor Domingues em 09-jul-2023
https://github.com/vitorshaft
https://instagram.com/shaftrobotica

Exemplo de utilização de controle Proporcional Integral-Derivativo (PID).
    Este exemplo se baseia no controle de 6 juntas de um braço robótico dotado de uma câmera em seu elemento terminal.
    Sendo assim, é feita a leitura da posição de um objeto na imagem da câmera (u,v) e a posição estimada no espaço (x6,z6).
    O vetor qd[] armazena a posição angular (em radianos) de cada junta
Você também pode importar este arquivo como um módulo e aproveitar a classe criada neste código.
'''

import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0.0
        self.error_sum = 0.0

    def compute(self, error, dt):
        # Termo proporcional
        P = self.Kp * error

        # Termo integral
        self.error_sum += error * dt
        I = self.Ki * self.error_sum

        # Termo derivativo
        D = self.Kd * (error - self.last_error) / dt

        # Atualiza o último erro
        self.last_error = error

        # Calcula o valor de controle total
        control = P + I + D

        return control


def servoVisual(entrada,estado,target,K):
    
    qd = estado
    u, v, z6, x6 = entrada[0], entrada[1], entrada[2], entrada[3]
    # Configuração dos controladores PID para cada variável

    pid_u = PIDController(Kp=K[0], Ki=K[1], Kd=K[2])
    pid_v = PIDController(Kp=K[0], Ki=K[1], Kd=K[2])
    pid_z = PIDController(Kp=K[0], Ki=K[1], Kd=K[2])
    pid_w = PIDController(Kp=K[0], Ki=K[1], Kd=K[2])
    # Definir as metas para cada variável (valores desejados)
    target_u = target[0]
    target_v = target[1]
    target_z = target[2]
    target_w = target[3]

    # Variáveis de erro acumuladas para o controle PID
    error_sum_u = 0.0
    error_sum_v = 0.0
    error_sum_z = 0.0
    error_sum_w = 0.0

    # Variáveis de controle para as saídas do PID
    control_u = 0.0
    control_v = 0.0
    control_z = 0.0
    control_w = 0.0

    # Limite de controle máximo para as saídas do PID
    control_max = 1.0

    # Calcula os erros para cada variável
    error_u = target_u - u
    error_v = target_v - v
    error_z = target_z - z6
    error_w = target_w - x6

    # Acumula os erros para uso na parte integral do controle PID
    error_sum_u += error_u
    error_sum_v += error_v
    error_sum_z += error_z
    error_sum_w += error_w

    # Calcula as saídas do controle PID para cada variável
    control_u = pid_u.compute(error_u, error_sum_u)
    control_v = pid_v.compute(error_v, error_sum_v)
    control_z = pid_z.compute(error_z, error_sum_z)
    control_w = pid_w.compute(error_w, error_sum_w)
    # Limita as saídas do controle dentro do intervalo permitido
    control_u = np.clip(control_u, -control_max, control_max)
    control_v = np.clip(control_v, -control_max, control_max)
    control_z = np.clip(control_z, -control_max, control_max)
    control_w = np.clip(control_w, -control_max, control_max)

    # Atualiza as variáveis de controle qd com base nas saídas do controle PID
    qd[0] += control_u
    qd[1] += control_z
    qd[2] -= control_v
    qd[3] -= control_w
    qd[4] -= control_z
    qd[5] -= control_u

    return(qd)

# Vetor de variáveis de estado
qd = [0.0, 1.5, 3.0, 4.5, 6.0, 7.5]
# Variáveis de entrada (aleatórios)
u, v, z6, x6 = 0.1, 0.2, 0.3, 0.4
entrada = [u, v, z6, x6]
# Vetor de valores objetivos (Os valores que você quer alcançar)
target = [0.5, 0.5, 0.15, 0.06]
#Constantes de proporcionalidade (Kp, Ki e Kd)
K = [0.1, 0.01, 0.05]

servoVisual(entrada,qd,)