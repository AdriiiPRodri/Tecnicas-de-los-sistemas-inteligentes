(define (domain Ejercicio1-domain)	       ; Comment: adding location caused fail
  (:requirements :strips :equality :typing)
  (:types orientation player npc object zone surface)

  (:predicates
	       (actualOrientation ?a - orientation ?p - player)
	       (orientedZone ?iniZ - zone ?o - orientation ?endZ - zone)
	       (atplayer ?p - player ?z - zone)
	       (atNPC ?p - npc ?z - zone)
	       (atObject ?o - object ?z - zone)
	       (hasObjectNPC ?n - npc ?o -object)
         (typesurface ?z - zone ?s - surface)
         (bagObject ?o - object)
         (canTakeObject)
         (canPutObject)
         (hasObjectPlayer ?p - player ?o - object)
  )

  (:functions
         (distance ?x ?y - zone)
         (total-distance)
         (objectScore ?o - object ?n - npc)
         (totalScore)
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
	            (orientedZone ?iniZone ?actualO ?endZone)
	            (atplayer ?p ?iniZone)
              (not (typesurface ?endZone bosque))
              (not (typesurface ?endZone agua))
              (not (typesurface ?endZone precipicio))
	     )
	     :effect
	     (
	        and
	        	(increase (total-distance) (distance ?iniZone ?endZone))
            (atplayer ?p ?endZone)
            (not(atplayer ?p ?iniZone))
	     )
  )

  (:action MOVE-FORWARD-WATER
       :parameters (?actualO - orientation ?p - player ?iniZone ?endZone - zone)
       :precondition
       (
          and
              (actualOrientation ?actualO ?p)
              (orientedZone ?iniZone ?actualO ?endZone)
              (atplayer ?p ?iniZone)
              (typesurface ?endZone agua)
              (or (hasObjectPlayer ?p bikini) (bagObject bikini))
       )
       :effect
       (
          and
            (increase (total-distance) (distance ?iniZone ?endZone))
            (atplayer ?p ?endZone)
            (not(atplayer ?p ?iniZone))
       )
  )

  (:action MOVE-FORWARD-FOREST
       :parameters (?actualO - orientation ?p - player ?iniZone ?endZone - zone)
       :precondition
       (
          and
              (actualOrientation ?actualO ?p)
              (orientedZone ?iniZone ?actualO ?endZone)
              (atplayer ?p ?iniZone)
              (typesurface ?endZone bosque)
              (or (hasObjectPlayer ?p zapatilla) (bagObject zapatilla))
       )
       :effect
       (
          and
            (increase (total-distance) (distance ?iniZone ?endZone))
            (atplayer ?p ?endZone)
            (not(atplayer ?p ?iniZone))
       )
  )

  (:action TAKE-OBJECT
    :parameters (?o - object ?p - player ?z - zone)
    :precondition
    (
      and
        (canTakeObject)
        (not (hasObjectPlayer ?p ?o))
        (atPlayer ?p ?z)
        (atObject ?o ?z)
    )
    :effect
    (
      and
        (not (canTakeObject))
        (hasObjectPlayer ?p ?o)
        (not (atObject ?o ?z))
    )
  )

  (:action PUT-OBJECT-BAG
    :parameters (?o - object ?p - player)
    :precondition
    (
      and
        (canPutObject)
        (hasObjectPlayer ?p ?o)
    )
    :effect
    (
      and
        (canTakeObject)
        (not (hasObjectPlayer ?p ?o))
        (bagObject ?o)
        (not (canPutObject))
    )
  )

  (:action EXTRACT-OBJECT-BAG
    :parameters (?o - object ?p - player)
    :precondition
    (
      and
        (canTakeObject)
        (not (canPutObject))
        (bagObject ?o)
        (not (hasObjectPlayer ?p ?o))
    )
    :effect
    (
      and
        (not (canTakeObject))
        (hasObjectPlayer ?p ?o)
        (not (bagObject ?o))
        (canPutObject)
    )
  )

  (:action DROP-OBJECT
    :parameters (?o - object ?p - player ?z - zone)
    :precondition
    (
      and
        (not (canTakeObject))
        ;(not (canPutObject))
        (atPlayer ?p ?z)
        (hasObjectPlayer ?p ?o)
        (not (atObject oro ?z))     ; Para que no elimine un posible objeto que este en el suelo
        (not (atObject algoritmos ?z))
        (not (atObject rosas ?z))
        (not (atObject oscars ?z))
        (not (atObject manzanas ?z))
        (not (atObject zapatilla ?z))
        (not (atObject bikini ?z))
    )
    :effect
    (
      and
        (canTakeObject)
        (not (hasObjectPlayer ?p ?o))
        ;(not (bagObject ?o))
        (atObject ?o ?z)
        ;(canPutObject)
    )
  )

  (:action GIVE-OBJECT
       :parameters (?z - zone ?o - object ?p - player ?n - npc)
	     :precondition
	     (
	        and
	            (hasObjectPlayer ?p ?o)
	            (atplayer ?p ?z)
	            (atNPC ?n ?z)
	            (not(hasObjectNPC ?n ?o))
              (not (hasObjectPlayer ?p bikini))   ; Si tiene este objeto se lo daría por tanto necesitamos que no se lo de pues nos hace falta
              (not (hasObjectPlayer ?p zapatilla)); Si tiene este objeto se lo daría por tanto necesitamos que no se lo de pues nos hace falta
	     )
	     :effect
	     (
	        and
              (canTakeObject)
              (not (hasObjectPlayer ?p ?o))
              (hasObjectNPC ?n ?o)
              (increase (totalScore) (objectScore ?o ?n))
	     )
  )
)
