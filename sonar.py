"""Mesure des deux sonars et signalisation par DEL."""

import math
import threading
import time

from ev_app_client_api import fermer_client, gen_ev_externe
from param import (
    APP_LIGNE,
    DEL_JAUNE_GPIO,
    DEL_VERTE_GPIO,
    DISTANCE_SONAR_MAX_CM,
    DISTANCE_SONAR_MIN_CM,
    FENETRE_LISSAGE_SONAR,
    INTERVALLE_MESURE_SONAR,
    MSG_SONAR,
    PERIODE_DEL_LENTE_S,
    PERIODE_DEL_MIN_S,
    PERIODE_DEL_MOYENNE_S,
    PERIODE_DEL_RAPIDE_S,
    SEUIL_SONAR_MESSAGE_CM,
    SEUIL_SONAR_RAPIDE_CM,
    SONAR_DROIT_ECHO_GPIO,
    SONAR_DROIT_TRIGGER_GPIO,
    SONAR_GAUCHE_ECHO_GPIO,
    SONAR_GAUCHE_TRIGGER_GPIO,
    VITESSE_SON_CM_S,
)


def _charger_gpiozero():
    try:
        from gpiozero import DigitalInputDevice, DigitalOutputDevice
    except ImportError as erreur:
        raise SystemExit("gpiozero est necessaire sur le Pi") from erreur
    return DigitalInputDevice, DigitalOutputDevice


class Signaleur:

    def __init__(self, gpio_del, sortie_factory=None):
        if sortie_factory is None:
            _, sortie_factory = _charger_gpiozero()

        self._del = sortie_factory(gpio_del)
        self._periode = PERIODE_DEL_LENTE_S
        self._arret = None
        self._fil = None
        self._ferme = False
        self._verrou_operation = threading.Lock()

    @property
    def periode(self):
        return self._periode

    def _executer(self, arret, periode):
        demi_periode = periode / 2.0
        try:
            while not arret.is_set():
                self._del.on()
                if arret.wait(demi_periode):
                    break
                self._del.off()
                if arret.wait(demi_periode):
                    break
        finally:
            self._del.off()

    def _arreter_fil(self):
        arret = self._arret
        fil = self._fil
        self._arret = None
        self._fil = None

        if arret is not None:
            arret.set()
        if fil is not None and fil is not threading.current_thread():
            fil.join()
        self._del.off()

    def demarrer(self):
        with self._verrou_operation:
            if self._ferme:
                raise RuntimeError("Le signaleur est fermer")
            if self._fil is not None and self._fil.is_alive():
                return

            self._arret = threading.Event()
            self._fil = threading.Thread(
                target=self._executer,
                args=(self._arret, self._periode),
                name="signaleur-del",
                daemon=True,
            )
            self._fil.start()

    def arreter(self):
        with self._verrou_operation:
            self._arreter_fil()

    def clignoter(self, periode):
        periode = float(periode)
        if not math.isfinite(periode):
            raise ValueError("La periode de clignotement doit etre finie")
        periode = max(PERIODE_DEL_MIN_S, periode)

        with self._verrou_operation:
            if self._ferme:
                raise RuntimeError("Le signaleur est fermer")
            self._arreter_fil()
            self._periode = periode
            self._arret = threading.Event()
            self._fil = threading.Thread(
                target=self._executer,
                args=(self._arret, self._periode),
                name="signaleur-del",
                daemon=True,
            )
            self._fil.start()

    def fermer(self):
        with self._verrou_operation:
            if self._ferme:
                return
            self._arreter_fil()
            self._del.close()
            self._ferme = True


class Lisseur:
    def __init__(self, grandeur_fenetre):
        grandeur_fenetre = int(grandeur_fenetre)
        if grandeur_fenetre <= 0:
            raise ValueError("La grandeur de la fenetre doit etre positive")

        self.grandeur_fenetre = grandeur_fenetre
        self._valeurs = []
        self._verrou = threading.Lock()

    def _ajouter(self, valeur):
        valeur = float(valeur)
        if not math.isfinite(valeur):
            raise ValueError("La valeur a lisser doit etre finie")
        self._valeurs.append(valeur)
        if len(self._valeurs) > self.grandeur_fenetre:
            del self._valeurs[0]
        return list(self._valeurs)

    def lisser(self, valeur):
        with self._verrou:
            valeurs = self._ajouter(valeur)
            return sum(valeurs) / len(valeurs)

    def lisser_min_max(self, valeur):
        with self._verrou:
            valeurs = self._ajouter(valeur)
            if len(valeurs) >= 3:
                valeurs.remove(min(valeurs))
                valeurs.remove(max(valeurs))
            return sum(valeurs) / len(valeurs)


class Sonar:
    def __init__(
        self,
        gpio_trigger,
        gpio_echo,
        gpio_del,
        envoyer=gen_ev_externe,
        horloge=time.perf_counter,
        dormir=time.sleep,
        sortie_factory=None,
        entree_factory=None,
        signaleur=None,
        grandeur_fenetre=FENETRE_LISSAGE_SONAR,
        identifiant=None,
    ):
        if sortie_factory is None or entree_factory is None:
            entree_gpiozero, sortie_gpiozero = _charger_gpiozero()
            sortie_factory = sortie_factory or sortie_gpiozero
            entree_factory = entree_factory or entree_gpiozero

        self._trigger = sortie_factory(gpio_trigger)
        self._echo = entree_factory(gpio_echo, pull_up=False)
        self._signaleur = signaleur or Signaleur(
            gpio_del,
            sortie_factory=sortie_factory,
        )
        self._lisseur = Lisseur(grandeur_fenetre)
        self._envoyer = envoyer
        self._horloge = horloge
        self._dormir = dormir
        self._identifiant = identifiant
        self._debut_echo = None
        self._mesure_en_attente = False
        self._periode_signaleur = None
        self._obstacle_signale = False
        self._arrete = False
        self.derniere_distance = None

        self._trigger.off()
        self._echo.when_activated = self._front_montant
        self._echo.when_deactivated = self._front_descendant

    def _front_montant(self, _entree=None):
        if not self._arrete and self._mesure_en_attente:
            self._debut_echo = self._horloge()

    def _front_descendant(self, _entree=None):
        fin_echo = self._horloge()
        if (
            self._arrete
            or not self._mesure_en_attente
            or self._debut_echo is None
        ):
            return
        duree = fin_echo - self._debut_echo
        self._debut_echo = None
        self._mesure_en_attente = False

        distance = VITESSE_SON_CM_S * duree / 2.0
        if not math.isfinite(distance) or distance <= 0:
            return

        distance = max(
            DISTANCE_SONAR_MIN_CM,
            min(DISTANCE_SONAR_MAX_CM, distance),
        )
        self._traiter_distance(distance)

    def _traiter_distance(self, distance):
        distance = self._lisseur.lisser_min_max(distance)
        self.derniere_distance = distance
        self._actualiser_signaleur(distance)
        self._transmettre_obstacle(distance)

    def _actualiser_signaleur(self, distance):
        if distance < SEUIL_SONAR_RAPIDE_CM:
            periode = PERIODE_DEL_RAPIDE_S
        elif distance < SEUIL_SONAR_MESSAGE_CM:
            periode = PERIODE_DEL_MOYENNE_S
        else:
            periode = PERIODE_DEL_LENTE_S

        if periode != self._periode_signaleur:
            self._signaleur.clignoter(periode)
            self._periode_signaleur = periode

    def _transmettre_obstacle(self, distance):
        obstacle_detecte = distance < SEUIL_SONAR_MESSAGE_CM

        if not obstacle_detecte and not self._obstacle_signale:
            return

        try:
            donnees = [round(distance, 2)]
            if self._identifiant is not None:
                donnees.append(self._identifiant)
            self._envoyer("127.0.0.1", APP_LIGNE, MSG_SONAR, *donnees)

            if obstacle_detecte and not self._obstacle_signale:
                nom_sonar = self._identifiant or "sonar"
                print(
                    f"MSG_SONAR({distance:.2f}) transmis par "
                    f"le sonar {nom_sonar}: obstacle sous 50 cm."
                )
            self._obstacle_signale = obstacle_detecte
        except OSError as erreur:
            print(f"MSG_SONAR non transmis a ligne.py: {erreur}")

    def mesurer(self):
        if self._arrete:
            return
        mesure_sans_echo = self._mesure_en_attente
        self._mesure_en_attente = True
        self._debut_echo = None

        if mesure_sans_echo:
            self._traiter_distance(DISTANCE_SONAR_MAX_CM)

        self._trigger.on()
        try:
            self._dormir(0.000010)
        finally:
            self._trigger.off()

    def arreter(self):
        if self._arrete:
            return
        self._arrete = True
        self._debut_echo = None
        self._mesure_en_attente = False

        try:
            fermer = getattr(self._signaleur, "fermer", None)
            if fermer is not None:
                fermer()
            else:
                self._signaleur.arreter()
        finally:
            self._trigger.off()
            self._trigger.close()
            self._echo.close()


def main():
    sonar_gauche = Sonar(
        SONAR_GAUCHE_TRIGGER_GPIO,
        SONAR_GAUCHE_ECHO_GPIO,
        DEL_JAUNE_GPIO,
        identifiant="gauche",
    )
    sonar_droit = Sonar(
        SONAR_DROIT_TRIGGER_GPIO,
        SONAR_DROIT_ECHO_GPIO,
        DEL_VERTE_GPIO,
        identifiant="droit",
    )

    print("Sonars actifs a 10 mesures par seconde.")
    try:
        while True:
            debut_cycle = time.perf_counter()
            sonar_gauche.mesurer()
            time.sleep(INTERVALLE_MESURE_SONAR / 2.0)
            sonar_droit.mesurer()
            reste = INTERVALLE_MESURE_SONAR - (
                time.perf_counter() - debut_cycle
            )
            if reste > 0:
                time.sleep(reste)
    except KeyboardInterrupt:
        pass
    finally:
        sonar_gauche.arreter()
        sonar_droit.arreter()
        fermer_client()
        print("Programme sonar arrete; GPIO et DEL desactives.")


if __name__ == "__main__":
    main()
