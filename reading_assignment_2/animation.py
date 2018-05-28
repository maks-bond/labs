import cozmo

async def run(robot: cozmo.robot.Robot):
	#anim_bored_event_02
	#anim_pounce_success_02
	#anim_poked_giggle
	await robot.play_anim(name='anim_bored_event_02').wait_for_completed()

if __name__ == '__main__':
    cozmo.run_program(run)