import math

#---------------------------------------> PARAMETROS <--------------------------------------------
   
V_LINEAL = 0.2    # velocidad por defecto en rectas y arcos [m/s]
V_GIRO = 0.5      # velocidad por defecto en giros sobre si mismo [rad/s]

#-------------------------------------------------------------------------------------------------
   
class Trayectoria:

    def __init__(self, rumbo=0.0):
        self.tramos = []                    # (duracion, funcion(tau) -> estado relativo)
        self.x = self.y = self.theta = 0.0  # estado al final de la cadena
        self.rumbo = math.radians(rumbo)    # direccion de avance inicial (para los arcos), grados

    #------------------------------------> MOVIMIENTOS <-------------------------------------------
    def recta(self, dx, dy, v=V_LINEAL, girar=0.0):
        
        # Avanza (dx, dy) metros en linea recta
        # Girar: el robot rota esos grados mientras avanza.
        largo = math.hypot(dx, dy)
        if largo == 0:
            return self
        T = largo / v
        dth = math.radians(girar)
        self._agregar(T, lambda tau: (dx * tau / T, dy * tau / T, dth * tau / T,
                                      dx / T, dy / T, dth / T))
        self.rumbo = math.atan2(dy, dx)
        return self

    def diagonal(self, distancia, angulo, v=V_LINEAL, girar=0.0):
        
        # Recta de `distancia` metros en la direccion `angulo` 
        # Grados: 0 = adelante, 90 = izquierda, 45 = adelante-izquierda
        a = math.radians(angulo)
        return self.recta(distancia * math.cos(a), distancia * math.sin(a), v, girar)

    def arco(self, radio, angulo, v=V_LINEAL, girar=0.0):
        # Curva de `angulo` grados positivo a la izquierda, negativo a la derecha con el radio dado
        a = math.radians(angulo)
        T = radio * abs(a) / v
        sentido = math.copysign(1, a)
        r0 = self.rumbo
        # Centro del arco, a la izquierda (o derecha) de la direccion de avance
        cx, cy = -sentido * radio * math.sin(r0), sentido * radio * math.cos(r0)
        dth = math.radians(girar)

        def estado(tau):
            phi = r0 + a * tau / T                  # direccion de avance en tau
            w = a / T
            return (cx + sentido * radio * math.sin(phi), cy - sentido * radio * math.cos(phi),
                    dth * tau / T,
                    sentido * radio * w * math.cos(phi), sentido * radio * w * math.sin(phi),
                    dth / T)
        self._agregar(T, estado)
        self.rumbo = r0 + a
        return self

    def giro(self, angulo, w=V_GIRO):
        # Gira 'angulo' grados sobre si mismo, sin moverse
        a = math.radians(angulo)
        T = abs(a) / w
        self._agregar(T, lambda tau: (0.0, 0.0, a * tau / T, 0.0, 0.0, a / T))
        return self

    def pausa(self, segundos):
        # Se queda quieto.
        self._agregar(segundos, lambda tau: (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        return self

    #---------------------------------------------------------------------------------------------
    
    #------------------------------------> EVALUAR <----------------------------------------------
    @property
    def duracion(self):
        return sum(T for T, _ in self.tramos)

    def __call__(self, t):
        # Estado deseado en el instante t: (x, y, theta, dx, dy, dtheta).
        for T, estado in self.tramos:
            if t <= T:
                return estado(t)
            t -= T
        return self.x, self.y, self.theta, 0.0, 0.0, 0.0

    def _agregar(self, T, relativo):
        x0, y0, th0 = self.x, self.y, self.theta

        def estado(tau):
            x, y, th, dx, dy, dth = relativo(tau)
            return x0 + x, y0 + y, th0 + th, dx, dy, dth
        self.tramos.append((T, estado))
        self.x, self.y, self.theta = estado(T)[:3]
    #---------------------------------------------------------------------------------------------
    

# --- Figuras de ejemplo (caben en la arena de 6 x 6 m partiendo del inicio, en (0, -1.5)) -----
FIGURAS = {
    'cuadrado': lambda: Trayectoria().recta(1.2, 0).recta(0, 1.2).recta(-1.2, 0).recta(0, -1.2),
    'cuadrado_2m': lambda: Trayectoria().recta(2.0, 0).recta(0, 2.0).recta(-2.0, 0).recta(0, -2.0),
    'cuadrado_girando': lambda: (Trayectoria().recta(1.2, 0, girar=90).recta(0, 1.2, girar=90)
                                 .recta(-1.2, 0, girar=90).recta(0, -1.2, girar=90)),
    'rombo': lambda: Trayectoria().diagonal(1.2, 45).diagonal(1.2, 135).diagonal(1.2, -135).diagonal(1.2, -45),
    'circulo': lambda: Trayectoria().arco(0.8, 360),
    'ocho': lambda: Trayectoria(rumbo=90).arco(0.6, 360).arco(0.6, -360),   # arranca hacia +y
    'demo': lambda: (Trayectoria().recta(1.0, 0).giro(90).arco(0.5, 180).diagonal(0.7, -45)
                     .recta(0, -0.5, girar=-90)),
    
}

#------------------------------------------------------------------------------------------------
