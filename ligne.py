#!/bin/python3

"""Arret lorsque le robot est a un metre du debut de la ligne."""

import math

from ev_app import EvApp
from ev_app_client_api import fermer_client, gen_ev_externe
from param import (
    APP_CTRL_ROBOT,
    APP_LIGNE,
    DISTANCE_LIGNE_CM,
    DISTANCE_PAR_TRANSITION_CM,
    MSG_ARRETER,
    MSG_AVANCER,
    MSG_INIT,
    MSG_POSITION,
    MSG_SONAR,
    MSG_BLOQUE,
    SEUIL_SONAR_ARRET_CM,
    SEUIL_SONAR_RAPIDE_CM,
    VITESSE_INITIALE,
)


class Ligne(EvApp):
    def __init__(
        self,
        port_no=APP_LIGNE,
        envoyer=gen_ev_externe,
        afficher=print,
        **app_options,
    ):
        super().__init__(port_no, **app_options)
        self._envoyer = envoyer
        self._afficher = afficher
        self.distance_parcourue = 0.0
        self._point_initial = None
        self._attend_position_initiale = True
        self._obstacle = False
        self._distance_max_atteinte = False
        self._distances_sonars = {}

        self._demander_initialisation()

    @staticmethod
    def lire_mesure_sonar(evenement):
        donnees = evenement.split()
        if len(donnees) not in (1, 2) or donnees[0] == "":
            raise ValueError("MSG_SONAR doit contenir une distance")

        try:
            distance = float(donnees[0])
        except (TypeError, ValueError) as erreur:
            raise ValueError(
                "MSG_SONAR ne contient pas un nombre"
            ) from erreur

        if not math.isfinite(distance) or distance < 0:
            raise ValueError("MSG_SONAR contient une distance invalide")

        identifiant = donnees[1].strip() if len(donnees) == 2 else "sonar"
        if not identifiant:
            raise ValueError("MSG_SONAR contient un identifiant vide")
        return distance, identifiant

    @staticmethod
    def lire_distance_sonar(evenement):
        distance, _identifiant = Ligne.lire_mesure_sonar(evenement)
        return distance

    def _envoyer_controleur(self, type_message, *donnees):
        try:
            self._envoyer(
                "127.0.0.1",
                APP_CTRL_ROBOT,
                type_message,
                *donnees,
            )
            return True
        except OSError as erreur:
            print(f"Message non transmis au robot: {erreur}")
            return False

    @staticmethod
    def lire_position(evenement):
        donnees = evenement.split()
        if len(donnees) != 3:
            raise ValueError(
                "MSG_POSITION doit contenir x, y et l'orientation"
            )

        try:
            valeurs = tuple(float(donnee) for donnee in donnees)
        except (TypeError, ValueError) as erreur:
            raise ValueError(
                "MSG_POSITION ne contient pas de nombres"
            ) from erreur

        if not all(math.isfinite(valeur) for valeur in valeurs):
            raise ValueError("MSG_POSITION ne contient pas de nombres finis")
        return valeurs

    def _demander_initialisation(self):
        self.distance_parcourue = 0.0
        self._point_initial = None
        self._attend_position_initiale = True
        self._envoyer_controleur(MSG_INIT)

    def _accepter_position_initiale(self, x, y):
        distance_origine = math.hypot(x, y)
        if distance_origine > DISTANCE_PAR_TRANSITION_CM:
            return False

        self._point_initial = (x, y)
        self._attend_position_initiale = False
        self._afficher("Nouvelle ligne: position initiale.")
        return True

    def _calculer_distance(self, x, y):
        x_initial, y_initial = self._point_initial
        self.distance_parcourue = math.hypot(
            x - x_initial,
            y - y_initial,
        )

        if self.distance_parcourue < DISTANCE_LIGNE_CM:
            return

        self._afficher(
            f"Distance atteinte: {self.distance_parcourue:.2f} cm. "
            "Robot arrete et odometrie reinitialisee."
        )
        self._distance_max_atteinte = True
        self._demander_initialisation()

    def _envoyer_obstacle(self, obstacle, distance):
        if self._obstacle != obstacle:
            self._obstacle = obstacle
            if self._envoyer_controleur(MSG_BLOQUE, obstacle):
                if obstacle:
                    self._afficher(
                        f"Obstacle a {distance:.2f} cm: "
                        "robot bloqué."
                    )
                else:
                    self._afficher(
                        f"Zone degagée ({distance:.2f} cm): "
                        "robot débloqué."
                    )

    def _traiter_sonar(self, evenement):
        try:
            distance, identifiant = self.lire_mesure_sonar(evenement)
        except ValueError as erreur:
            print(f"MSG_SONAR invalide ignore: {erreur}")
            return

        self._distances_sonars[identifiant] = distance
        distance_minimale = min(self._distances_sonars.values())
        self._envoyer_obstacle(distance_minimale < SEUIL_SONAR_ARRET_CM, distance_minimale)

    def dispatch_event(self, evenement):
        if evenement is None:
            return
        if evenement.type == MSG_SONAR:
            self._traiter_sonar(evenement)
            return
        if evenement.type != MSG_POSITION:
            print(f"Message inconnu ignore: {evenement.type}")
            return

        try:
            x, y, _angle = self.lire_position(evenement)
        except ValueError as erreur:
            print(f"MSG_POSITION invalide ignore: {erreur}")
            return

        if self._attend_position_initiale:
            self._accepter_position_initiale(x, y)
            return
        self._calculer_distance(x, y)

    def quitter(self):
        self._demander_initialisation()
        print("Programme ligne arrete; reinitialiser.")


def main():
    ligne = Ligne()
    print(
        f"Programme ligne en ecoute sur le port local {APP_LIGNE}."
    )
    try:
        ligne.run()
    finally:
        fermer_client()


if __name__ == "__main__":
    main()
