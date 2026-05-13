class RadarSystem:

    def __init__(self, projectile):
      self.projectile = projectile
  
    def get_target_position(self):
      return self.projectile.getPosition()