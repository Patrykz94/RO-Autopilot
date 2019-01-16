import os

# Please enter the mission parameters below. You can enter "default" for any of the fields to use default values
# Default orbit is 250 x 250 km, everything else "unlocked"
apogee = "250"                      # Apoapsis in km
perigee = "250"                     # Periapsis in km
inclination = "default"             # Inclination in degrees.
lan = "default"                     # Longitude of Ascending Node in degrees.
trueAnomaly = "default"             # True anomaly in degrees.
landing = "EXP"                     # Options: "RTLS", "ASDS", "EXP"


def run_mission(mission_name):
    mission = str(mission_name).capitalize()
    if mission_name == "Launch":
        mission = "Launch"
        our_mission = list()
        our_mission.append(mission)
        our_mission.append(apogee)
        our_mission.append(perigee)
        our_mission.append(inclination)
        our_mission.append(lan)
        our_mission.append(trueAnomaly)
        our_mission.append(landing.upper())
        our_mission = ' '.join(our_mission)
        os.system("python -m " + (our_mission))


if __name__ == '__main__':
    run_mission("Launch")
