from mission import *


class TestMission():

    def test_should_create_missions(self):
        missions = Missions()
        aux = missions.get_missions()
        assert len(aux) == 0

    def test_should_add_a_mission(self):
        missions = Missions()
        missions.add_mission("New Mission", 0)
        aux = missions.get_mission(0)
        assert aux.name == "New Mission"
        assert len(aux.locations) == 0

    def test_should_add_a_mission_and_a_location(self):
        missions = Missions()
        missions.add_mission("New Mission", 0)
        aux = missions.get_mission(0)
        aux.add_location(-29.3512, -40.1333, 0, "ligar os motores")
        loc = missions.get_mission(0).get_location(0)
        assert loc.action == "ligar os motores"
        assert loc.latitude == -29.3512
        assert loc.longitude == -40.1333

    def test_should_include_mission_in_correct_order(self):
        missions = Missions()
        missions.add_mission("Mission 0", 0)
        missions.add_mission("Mission 1 (old)", 1)
        missions.add_mission("Mission 1 (new)", 1)
        assert missions.get_mission(1).name == "Mission 1 (new)"
        assert missions.get_mission(2).name == "Mission 1 (old)"
