"""Commande du moteur du robot."""

import math

from param import PWM_MAX, PWM_MIN


class Moteur:
    def __init__(self, pwm, pin_avant, pin_arriere):
        self._pwm = pwm
        self._pin_avant = pin_avant
        self._pin_arriere = pin_arriere
        self._sens = 0
        self.arreter()

    @staticmethod
    def limiter_puissance(puissance):
        puissance = float(puissance)
        if not math.isfinite(puissance):
            raise ValueError("La puissance doit etre finie")
        return max(PWM_MIN, min(PWM_MAX, puissance))

    def avancer(self, puissance):
        self._commander(puissance, 1)

    def reculer(self, puissance):
        self._commander(puissance, -1)

    def _commander(self, puissance, sens):
        puissance = self.limiter_puissance(puissance)
        if puissance == 0:
            self.arreter()
            return
        if sens != self._sens:
            self._pwm.value = PWM_MIN
            self._pin_avant.off()
            self._pin_arriere.off()
            if sens == 1:
                self._pin_avant.on()
            else:
                self._pin_arriere.on()
            self._sens = sens
        self._pwm.value = puissance

    def arreter(self):
        self._pwm.value = PWM_MIN
        self._pin_avant.off()
        self._pin_arriere.off()
        self._sens = 0

    def fermer(self):
        self.arreter()
        self._pwm.close()
        self._pin_avant.close()
        self._pin_arriere.close()
