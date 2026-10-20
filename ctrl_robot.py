#!/bin/python3

"""Arbitre la telecommande et le suiveur, puis commande les moteurs."""

import math
import time

from ev_app import EvApp
from moteur import Moteur
from param import (
    APP_CTRL_ROBOT,
    INTERVALLE_CONTROLE,
    MOTEUR_DROIT_IN1,
    MOTEUR_DROIT_IN2,
    MOTEUR_DROIT_PWM,
    MOTEUR_GAUCHE_IN1,
    MOTEUR_GAUCHE_IN2,
    MOTEUR_GAUCHE_PWM,
    MSG_ARRETER,
    MSG_AVANCER,
    MSG_PIVOTER_D,
    MSG_PIVOTER_G,
    MSG_RECULER,
    MSG_VITESSE,
    VITESSE_MAX,
    VITESSE_MIN,
)
from robot import Robot


class CtrlRobot(EvApp):
    def __init__(self, port_no, robot, horloge=time.monotonic, **app_options):
        app_options.setdefault("tmo", INTERVALLE_CONTROLE)
        super().__init__(port_no, **app_options)
        self.robot = robot
        self.action = MSG_ARRETER
        self._actions = {
            MSG_ARRETER: self.robot.arreter,
            MSG_AVANCER: self.robot.avancer,
            MSG_RECULER: self.robot.reculer,
            MSG_PIVOTER_G: self.robot.pivoter_gauche,
            MSG_PIVOTER_D: self.robot.pivoter_droite,
        }
        self.robot.arreter()

    @staticmethod
    def lire_vitesse(evenement):
        donnees = evenement.split()
        if len(donnees) != 1 or not donnees[0]:
            raise ValueError("Une seule valeur de vitesse est requise")
        vitesse = float(donnees[0])
        if not math.isfinite(vitesse):
            raise ValueError("La vitesse doit etre finie")
        return max(VITESSE_MIN, min(VITESSE_MAX, vitesse))

    def _arreter(self):
        self.robot.arreter()
        self.action = MSG_ARRETER

    def _appliquer_mouvement(self, commande):
        changement = commande != self.action
        self._actions[commande]()
        self.action = commande
        if changement:
            print(f"Commande: {commande}, PWM: {self.robot.vitesse:.2f}.")

    def _traiter_commande(self, evenement):
        if evenement.type == MSG_VITESSE:
            self.robot.vitesse = self.lire_vitesse(evenement)
            self._actions[self.action]()
            print(f"Vitesse PWM: {self.robot.vitesse:.2f}.")
            return

        if evenement.type in self._actions:
            self._appliquer_mouvement(evenement.type)

    def dispatch_event(self, evenement):
        if evenement is None:
            return
        try:
            self._traiter_commande(evenement)
        except (TypeError, ValueError) as erreur:
            self._arreter()
            print(f"Commande invalide, robot arrete: {erreur}")

    def quitter(self):
        self.action = MSG_ARRETER
        self.robot.fermer()
        print("Controleur arrete; moteurs desactives.")


def creer_controleur(port_no=APP_CTRL_ROBOT):
    try:
        from gpiozero import DigitalOutputDevice, PWMOutputDevice
    except ImportError as erreur:
        raise SystemExit("gpiozero est necessaire sur le Raspberry Pi") from erreur

    peripheriques = []

    def creer_sortie(classe, broche):
        sortie = classe(broche)
        peripheriques.append(sortie)
        return sortie

    try:
        moteur_gauche = Moteur(
            creer_sortie(PWMOutputDevice, MOTEUR_GAUCHE_PWM),
            creer_sortie(DigitalOutputDevice, MOTEUR_GAUCHE_IN1),
            creer_sortie(DigitalOutputDevice, MOTEUR_GAUCHE_IN2),
        )
        moteur_droit = Moteur(
            creer_sortie(PWMOutputDevice, MOTEUR_DROIT_PWM),
            creer_sortie(DigitalOutputDevice, MOTEUR_DROIT_IN1),
            creer_sortie(DigitalOutputDevice, MOTEUR_DROIT_IN2),
        )
        robot = Robot(moteur_gauche, moteur_droit)
        return CtrlRobot(port_no, robot)
    except BaseException:
        for peripherique in reversed(peripheriques):
            peripherique.close()
        raise


def main():
    controleur = creer_controleur()
    print(f"Controleur du robot en ecoute sur le port {APP_CTRL_ROBOT}.")
    controleur.run()


if __name__ == "__main__":
    main()
