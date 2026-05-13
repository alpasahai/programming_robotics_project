class ProjectileSystem:

  def __init__(self,projectile):
    self.projectile = projectile
    
  def launch_projectile(self):
    #self.projectile.setVelocity([0, 5, 5, 0, 0, 0]) #z controls the height
    self.projectile.setVelocity([3, 5, 10, 0, 0, 0])

  def reset_projectile(self):
     translation_field = self.projectile.getField("translation")
     translation_field.setSFVec3f([0, 1, 0])
     self.projectile.resetPhysics()