try:
    from gpiozero import (
            DigitalInputDevice,
            DigitalOutputDevice,
            PWMOutputDevice
        )
except ImportError as erreur:
    raise SystemExit("gpiozero est necessaire sur le Pi") from erreur
