class ProjectileSystem:

#Intial position of projectile:  [2.59591e-05, -2.53, 0.04984303999999999]

  def __init__(self,projectile):
    self.projectile = projectile
    self.spawn_position = [0, 1.2, 0]
    
  def launch_projectile(self):
    #self.projectile.setVelocity([0, 5, 5, 0, 0, 0]) #z controls the height
    #self.projectile.setVelocity([3, 5, 10, 0, 0, 0])
    self.projectile.setVelocity([0, 4, 6, 0, 0, 0])

  def reset_projectile(self):
    #Making sure projectile isn't moving
    self.projectile.setVelocity([0,0,0,0,0,0])
    self.projectile.resetPhysics()
    translation_field = self.projectile.getField("translation")
 
    translation_field.setSFVec3f([0, 1, 0])
    #Spawn position
    translation_field.setSFVec3f(self.spawn_position)