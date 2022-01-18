/* Note from posting to Code Samples:
 * This cpp is from my sophomore year custom C++ project.
 * The given object controlled checkpoints and set up
 * levels whenever they started (is there an ink wave,
 * should it be moving, etc.). This particular file is Doxygen
 * complient.
 *
 * Sometimes notes are added to explain the code for posterity.
 * these are marked as Code Sample Notes (CSN)
 */

/*!
 *
 *	\file		SceneController.cpp
 *	\author		William Pritz
 *	\par		w.pritz\@digipen.edu
 *	\par		GAM200/250
 *	\par		Scroll Up Engine
 *
 *	\brief
 *		Contains the implementation of a scene controller:
 *		catches events and updates the scene accordingly
 *
// Copyright © 2020 DigiPen (USA) Corporation.
 */

#pragma once

////
// Includes
////
#include "SceneController.h"	// SceneController Header
#include "Input.h"				// Input singleton
#include "SceneManager.h"		// Scene Manager singleton
#include "Sound.h"				// Plays Sounds
#include "GameObjectManager.h"	// finding game objects
#include "InkWave.h"			// Controlling the Ink Wave Object
#include "CameraSystem.h"		// find the current camera
#include "Camera.h"				// manipulate the camera
#include "Checkpoint.h"			// false activation of level checkpoints
#include "EventHandler.h"
#include "Modifiables.h"

namespace KNTEngine {

	// static instantiations of important variables
	bool SceneController::checkpointFlag_ = false;
	Vec2 SceneController::playerPosition_;
	Vec2 SceneController::inkPosition_;

	/*
	 * CSN: our custom engine built template objects, and then copied them
	 * to make the objects that filled our scenes. Hence, this constructor
	 * that doesn't store any real data, as it's only called for the template
	 * object.
	 */
	 
	/*!
	 *	\brief
	 *		Constructor for a scene controller
	 */
	SceneController::SceneController() :
		Component(to_string<SceneController>()),
		winEventHandler_("Player Wins",
			[](const EventData &data, SceneController * controller)
			{
				return;
			},
			this),
		deathEventHandler_("Player Death",
			[](const EventData &data, SceneController * controller)
			{
				return;
			}, 
			this),
		checkpointEventHandler_("Checkpoint",
			[](const EventData &data, SceneController * controller)
			{
				return;
			},
			this),
		inputEventHandler_("Input: Key pressed",
			[](const EventData &data, SceneController * controller)
			{
				return;
			},
			this),
		sceneChangeEventHandler_("SceneManager Is Changing",
			[](const EventData &data, SceneController * controller)
			{
				return;
			}, 
			this),
		sceneLoadedEventHandler_("", [](const EventData &data, SceneController * controller) {}, this),
		playGameHandler_("", [](const EventData &data, SceneController * controller) {}, this)
	{
	}

	/*!
	 *	\brief
	 *		Destructor for a scene controller
	 */
	SceneController::~SceneController()
	{
		Sound::StopAllSounds();
	}


	/*!
	 *	\brief
	 *		copy constructor for a scene controller
	 *	\param copy
	 *		the scene controller to be copied from
	 */
	SceneController::SceneController(const SceneController &copy) :
		Component(copy),
		music_(copy.music_),
		collectible_(copy.collectible_),
		counter_(copy.counter_),
		playGame_(copy.playGame_),
		winEventHandler_("Player Wins",
		[](const EventData &data, SceneController * controller)
		{
			if (controller->Parent().GetObjectListType() != ObjectListType::ActiveObject)
			{
				return;
			}
			// If the player wins, remove any checkpoints and proc the win menu.
			controller->setCheckpointFlag(false);
			SceneManager::SetNext("WinMenu");
		},this),
		deathEventHandler_("Player Death",
		[](const EventData &data, SceneController * controller)
		{
			if (controller->Parent().GetObjectListType() != ObjectListType::ActiveObject)
			{
				return;
			}
		},this),
		checkpointEventHandler_("Checkpoint",
		[](const EventData &data, SceneController * controller)
		{
			if (controller->Parent().GetObjectListType() != ObjectListType::ActiveObject)
			{
				return;
			}
			
			// on checkpoint hit, save where the ink wave was, play a sound, and save which checkpoint it is
			GameObject* wave = GameObjectManager::GetByName("InkWave");
			if (wave)
			{
				inkPosition_ = wave->Pos();
			}
			Sound::PlaySound(controller->GetCollectibleSound(), "Checkpoint Hit");
			controller->SetPlayerPosition(data.Interpret<Vec2>());
			controller->setCheckpointFlag(true);
		},this),
		inputEventHandler_("Input: Key pressed",
		[](const EventData &data, SceneController * controller)
		{
			if (controller->Parent().GetObjectListType() != ObjectListType::ActiveObject)
			{
				return;
			}

			InputKey key = data.Interpret<InputKey>();

			// If we've pressed I (cheat code button), stop the ink wave
			if (key == InputKey(GLFW_KEY_I, isKeyboardKey, GLFW_MOD_CONTROL))
			{
				GameObject* inkwave = GameObjectManager::GetByName("InkWave");
				if (inkwave)
				{
					InkWave* inkComponent = inkwave->GetComponent<InkWave>();
					if (inkComponent)
					{
						bool newIsMoving = !(inkComponent->GetIsMoving());
						inkComponent->SetIsMoving(newIsMoving);
					}
				}
			}
		},this),
		sceneChangeEventHandler_("SceneManager Is Changing",
		[](const EventData &data, SceneController * controller)
		{
			if (controller->Parent().GetObjectListType() != ObjectListType::ActiveObject)
			{
				return;
			}
		},this),
		sceneLoadedEventHandler_("Scene Loaded",
		[](const EventData &data, SceneController* controller)
		{
			bool restartflag = data.Interpret<bool>();
			
			// play the level music
			Sound::PlaySound(controller->music_, "Level Controller");

			GameObject *player = GameObjectManager::GetByName("Player");

			// check if the checkpoint is active
			if (restartflag == false)
			{
				checkpointFlag_ = false;
			}
			
			// if the checkpoint is active, move the player to the checkpoint
			if (player)
			{
				if (checkpointFlag_)
				{
					GameObject* checkpoint = GameObjectManager::GetByName("Checkpoint");
					if (checkpoint)
					{
						Checkpoint * checkercheeses = checkpoint->GetComponent<Checkpoint>();
						if (checkercheeses)
						{
							checkercheeses->FalseActivate();
						}
					}
					player->SetPos(playerPosition_);

					// start the ink wave 50 units below the player
					GameObject* wave = GameObjectManager::GetByName("InkWave");
					if (wave)
					{
						wave->SetPos(Vec2(inkPosition_.x, inkPosition_.y - 50));
					}
				}
			}
			return;
		},this),
		playGameHandler_("PlayGame",
			[](const EventData &data, SceneController * controller)
			{
				controller->SetCounter(2);
			}, this)
	{

	}

	/*!
	 *	\brief
	 *		called when moved between active and inactive objects list
	 *	\param oldList
	 *		type of the list the object has been movedd off of
	 */
	void SceneController::OnListChange(ObjectListType oldList)
	{
		if (oldList == ObjectListType::ActiveObject)
		{
			Sound::StopAllSounds();
		}
		else if (Parent().GetObjectListType() == ObjectListType::ActiveObject)
		{
			Sound::PlaySound(music_, "Level Controller");
		}
	}

	std::string const& SceneController::GetMusicTrack() const
	{
		return music_;
	}

	void SceneController::SetMusicTrack(std::string const& music_track)
	{
		music_ = music_track;
	}

	void SceneController::Update(float dt)
	{
		if (counter_)
		{
			--counter_;
			if (counter_ <= 0)
			{
				SceneManager::SetNext(playGame_);
			}
		}
	}
	
	Modifiables<Component> & SceneController::GetModifiables()
	{
		static Modifiables<Component> modifiables
		{
			MakeNeo<&SceneController::GetMusicTrack, &SceneController::SetMusicTrack>("Music Track"),
			MakeNeo<&SceneController::collectible_>("Collectible Sound"),
			MakeNeo<&SceneController::playGame_>("First Level"),
		};
		return modifiables;
	}

	//// Namespace End /////////////////////////////////////////////

	// End of KNTEngine Namespace
}
