(define (domain Ejercicio1-domain)	       ; Comment: adding location caused fail
  (:requirements :strips :equality :typing)
  (:types orientation player npc object zone)

  (:predicates
	       (actualOrientation ?a - orientation ?p - player)
	       (orientedZone ?iniZ - zone ?o - orientation ?endZ - zone)
	       (atplayer ?p - player ?z - zone)
	       (atNPC ?p - npc ?z - zone)
	       (atObject ?o - object ?z - zone)
         (hasObjectPlayer)
         (hasObjectNPC ?n - npc)
  )

  (:action TURN-RIGHT
	     :parameters (?a - orientation ?p - player)
	     :precondition (actualOrientation ?a ?p)
	     :effect
	     (
	        and
	        (when(and (actualOrientation north ?p))
	            (and (actualOrientation east ?p))
	        )
	        (when(and (actualOrientation east ?p))
	            (and (actualOrientation south ?p))
	        )
	        (when(and (actualOrientation south ?p))
	            (and (actualOrientation west ?p))
	        )
	        (when(and (actualOrientation west ?p))
	            (and (actualOrientation north ?p))
	        )
	        (not(actualOrientation ?a ?p))
	     )
  )

  (:action TURN-LEFT
	     :parameters (?a - orientation ?p - player)
	     :precondition (actualOrientation ?a ?p)
	     :effect
	     (
	        and
	        (when(and (actualOrientation north ?p))
	            (and (actualOrientation west ?p))
	        )
	        (when(and (actualOrientation west ?p))
	            (and (actualOrientation south ?p))
	        )
	        (when(and (actualOrientation south ?p))
	            (and (actualOrientation east ?p))
	        )
	        (when(and (actualOrientation east ?p))
	            (and (actualOrientation north ?p))
	        )
	        (not(actualOrientation ?a ?p))
	     )
  )

  (:action MOVE-FORWARD
       :parameters (?actualO - orientation ?p - player ?iniZone ?endZone - zone)
	     :precondition
	     (
	        and
	            (actualOrientation ?actualO ?p)
	            (atplayer ?p ?iniZone)
	            (orientedZone ?iniZone ?actualO ?endZone)
	     )
	     :effect
	     (
	        and
            (atplayer ?p ?endZone)
            (not(atplayer ?p ?iniZone))
	     )
  )

  (:action TAKE-OBJECT
    :parameters (?o - object ?p - player ?z - zone)
    :precondition
    (
      and
        (not (hasObjectPlayer))
        (atPlayer ?p ?z)
        (atObject ?o ?z)
    )
    :effect
    (
      and
        (hasObjectPlayer)
        (not (atObject ?o ?z))
    )
  )

  (:action DROP-OBJECT
    :parameters (?o - object ?p - player ?z - zone)
    :precondition
    (
      and
        (hasObjectPlayer)
        (atPlayer ?p ?z)
    )
    :effect
    (
      and
        (not (hasObjectPlayer))
        (atObject ?o ?z)
    )
  )

  (:action GIVE-OBJECT
       :parameters (?z - zone ?o - object ?p - player ?n - npc)
	     :precondition
	     (
	        and
	            (hasObjectPlayer)
	            (atplayer ?p ?z)
	            (atNPC ?n ?z)
	            (not(hasObjectNPC ?n))
	     )
	     :effect
	     (
	        and
              (not (hasObjectPlayer))
              (hasObjectNPC ?n)
	     )
  )
)
