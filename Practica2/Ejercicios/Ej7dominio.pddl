(define (domain Ejercicio1-domain)	       ; Comment: adding location caused fail
  (:requirements :strips :equality :typing)
  (:types orientation main cooperant npc object zone surface)

  (:predicates
	       (actualOrientationMain ?a - orientation ?m - main)
         (actualOrientationCooperant ?a - orientation ?c - cooperant)
	       (orientedZone ?iniZ - zone ?o - orientation ?endZ - zone)
	       (atmain ?m - main ?z - zone)
         (atcooperant ?c - cooperant ?z - zone)
	       (atNPC ?p - npc ?z - zone)
	       (atObject ?o - object ?z - zone)
         (typesurface ?z - zone ?s - surface)
         (bagObjectMain ?m - main ?o - object)
         (bagObjectCooperant ?m - cooperant ?o - object)
         (canTakeObjectCooperant)
         (canPutObjectCooperant)
         (canTakeObjectMain)
         (canPutObjectMain)
         (hasObjectMain ?m - main ?o - object)
         (hasObjectCooperant ?c - cooperant ?o - object)
  )

  (:functions
         (distance ?x ?y - zone)
         (total-distance)
         (objectScore ?o - object ?n - npc)
         (totalScore)
         (npcObjects ?n)
         (npcObjectsTotal ?n)
  )

  (:action TURN-RIGHT-MAIN
	     :parameters (?a - orientation ?p - main)
	     :precondition (actualOrientationMain ?a ?p)
	     :effect
	     (
	        and
	        (when(and (actualOrientationMain north ?p))
	            (and (actualOrientationMain east ?p))
	        )
	        (when(and (actualOrientationMain east ?p))
	            (and (actualOrientationMain south ?p))
	        )
	        (when(and (actualOrientationMain south ?p))
	            (and (actualOrientationMain west ?p))
	        )
	        (when(and (actualOrientationMain west ?p))
	            (and (actualOrientationMain north ?p))
	        )
	        (not(actualOrientationMain ?a ?p))
	     )
  )

  (:action TURN-RIGHT-COOPERANT
	     :parameters (?a - orientation ?p - cooperant)
	     :precondition (actualOrientationCooperant ?a ?p)
	     :effect
	     (
	        and
	        (when(and (actualOrientationCooperant north ?p))
	            (and (actualOrientationCooperant east ?p))
	        )
	        (when(and (actualOrientationCooperant east ?p))
	            (and (actualOrientationCooperant south ?p))
	        )
	        (when(and (actualOrientationCooperant south ?p))
	            (and (actualOrientationCooperant west ?p))
	        )
	        (when(and (actualOrientationCooperant west ?p))
	            (and (actualOrientationCooperant north ?p))
	        )
	        (not(actualOrientationCooperant ?a ?p))
	     )
  )

  (:action TURN-LEFT-MAIN
	     :parameters (?a - orientation ?p - main)
	     :precondition (actualOrientationMain ?a ?p)
	     :effect
	     (
	        and
	        (when(and (actualOrientationMain north ?p))
	            (and (actualOrientationMain west ?p))
	        )
	        (when(and (actualOrientationMain west ?p))
	            (and (actualOrientationMain south ?p))
	        )
	        (when(and (actualOrientationMain south ?p))
	            (and (actualOrientationMain east ?p))
	        )
	        (when(and (actualOrientationMain east ?p))
	            (and (actualOrientationMain north ?p))
	        )
	        (not(actualOrientationMain ?a ?p))
	     )
  )

  (:action TURN-LEFT-COOPERANT
	     :parameters (?a - orientation ?p - cooperant)
	     :precondition (actualOrientationCooperant ?a ?p)
	     :effect
	     (
	        and
	        (when(and (actualOrientationCooperant north ?p))
	            (and (actualOrientationCooperant west ?p))
	        )
	        (when(and (actualOrientationCooperant west ?p))
	            (and (actualOrientationCooperant south ?p))
	        )
	        (when(and (actualOrientationCooperant south ?p))
	            (and (actualOrientationCooperant east ?p))
	        )
	        (when(and (actualOrientationCooperant east ?p))
	            (and (actualOrientationCooperant north ?p))
	        )
	        (not(actualOrientationCooperant ?a ?p))
	     )
  )

  (:action MOVE-FORWARD-MAIN
       :parameters (?actualO - orientation ?p - main ?iniZone ?endZone - zone)
	     :precondition
	     (
	        and
	            (actualOrientationMain ?actualO ?p)
	            (orientedZone ?iniZone ?actualO ?endZone)
	            (atmain ?p ?iniZone)
              (not (typesurface ?endZone bosque))
              (not (typesurface ?endZone agua))
              (not (typesurface ?endZone precipicio))
	     )
	     :effect
	     (
	        and
	        	(increase (total-distance) (distance ?iniZone ?endZone))
            (atmain ?p ?endZone)
            (not(atmain ?p ?iniZone))
	     )
  )

  (:action MOVE-FORWARD-COOPERANT
       :parameters (?actualO - orientation ?p - cooperant ?iniZone ?endZone - zone)
	     :precondition
	     (
	        and
	            (actualOrientationCooperant ?actualO ?p)
	            (orientedZone ?iniZone ?actualO ?endZone)
	            (atcooperant ?p ?iniZone)
              (not (typesurface ?endZone bosque))
              (not (typesurface ?endZone agua))
              (not (typesurface ?endZone precipicio))
	     )
	     :effect
	     (
	        and
	        	(increase (total-distance) (distance ?iniZone ?endZone))
            (atcooperant ?p ?endZone)
            (not(atcooperant ?p ?iniZone))
	     )
  )

  (:action MOVE-FORWARD-WATER-MAIN
       :parameters (?actualO - orientation ?p - main ?iniZone ?endZone - zone)
       :precondition
       (
          and
              (actualOrientationMain ?actualO ?p)
              (orientedZone ?iniZone ?actualO ?endZone)
              (atmain ?p ?iniZone)
              (typesurface ?endZone agua)
              (or (hasObjectMain ?p bikini) (bagObjectMain ?p bikini))
       )
       :effect
       (
          and
            (increase (total-distance) (distance ?iniZone ?endZone))
            (atmain ?p ?endZone)
            (not(atmain ?p ?iniZone))
       )
  )

  (:action MOVE-FORWARD-WATER-COOPERANT
       :parameters (?actualO - orientation ?p - cooperant ?iniZone ?endZone - zone)
       :precondition
       (
          and
              (actualOrientationCooperant ?actualO ?p)
              (orientedZone ?iniZone ?actualO ?endZone)
              (atcooperant ?p ?iniZone)
              (typesurface ?endZone agua)
              (or (hasObjectCooperant ?p bikini) (bagObjectCooperant ?p bikini))
       )
       :effect
       (
          and
            (increase (total-distance) (distance ?iniZone ?endZone))
            (atcooperant ?p ?endZone)
            (not(atcooperant ?p ?iniZone))
       )
  )

  (:action MOVE-FORWARD-FOREST-MAIN
       :parameters (?actualO - orientation ?p - main ?iniZone ?endZone - zone)
       :precondition
       (
          and
              (actualOrientationMain ?actualO ?p)
              (orientedZone ?iniZone ?actualO ?endZone)
              (atmain ?p ?iniZone)
              (typesurface ?endZone bosque)
              (or (hasObjectMain ?p zapatilla) (bagObjectMain ?p zapatilla))
       )
       :effect
       (
          and
            (increase (total-distance) (distance ?iniZone ?endZone))
            (atmain ?p ?endZone)
            (not(atmain ?p ?iniZone))
       )
  )

  (:action MOVE-FORWARD-FOREST-COOPERANT
       :parameters (?actualO - orientation ?p - cooperant ?iniZone ?endZone - zone)
       :precondition
       (
          and
              (actualOrientationCooperant ?actualO ?p)
              (orientedZone ?iniZone ?actualO ?endZone)
              (atcooperant ?p ?iniZone)
              (typesurface ?endZone bosque)
              (or (hasObjectCooperant ?p zapatilla) (bagObjectCooperant ?p zapatilla))
       )
       :effect
       (
          and
            (increase (total-distance) (distance ?iniZone ?endZone))
            (atcooperant ?p ?endZone)
            (not(atcooperant ?p ?iniZone))
       )
  )

  (:action TAKE-OBJECT-COOPERANT
    :parameters (?o - object ?p - cooperant ?z - zone)
    :precondition
    (
      and
        (canTakeObjectCooperant)
        (not (hasObjectCooperant ?p ?o))
        (atcooperant ?p ?z)
        (atObject ?o ?z)
    )
    :effect
    (
      and
        (not (canTakeObjectCooperant))
        (hasObjectCooperant ?p ?o)
        (not (atObject ?o ?z))
    )
  )

  (:action PUT-OBJECT-BAG-MAIN
    :parameters (?o - object ?p - main)
    :precondition
    (
      and
        (canPutObjectMain)
        (hasObjectMain ?p ?o)
    )
    :effect
    (
      and
        (canTakeObjectMain)
        (not (hasObjectMain ?p ?o))
        (bagObjectMain ?p ?o)
        (not (canPutObjectMain))
    )
  )

  (:action PUT-OBJECT-BAG-COOPERANT
    :parameters (?o - object ?p - cooperant)
    :precondition
    (
      and
        (canPutObjectCooperant)
        (hasObjectCooperant ?p ?o)
    )
    :effect
    (
      and
        (canTakeObjectCooperant)
        (not (hasObjectCooperant ?p ?o))
        (bagObjectCooperant ?p ?o)
        (not (canPutObjectCooperant))
    )
  )

  (:action EXTRACT-OBJECT-BAG-MAIN
    :parameters (?o - object ?p - main)
    :precondition
    (
      and
        (canTakeObjectMain)
        (not (canPutObjectMain))
        (bagObjectMain ?p ?o)
        (not (hasObjectMain ?p ?o))
    )
    :effect
    (
      and
        (not (canTakeObjectMain))
        (hasObjectMain ?p ?o)
        (not (bagObjectMain ?p ?o))
        (canPutObjectMain)
    )
  )

  (:action EXTRACT-OBJECT-BAG-COOPERANT
    :parameters (?o - object ?p - cooperant)
    :precondition
    (
      and
        (canTakeObjectCooperant)
        (not (canPutObjectCooperant))
        (bagObjectCooperant ?p ?o)
        (not (hasObjectCooperant ?p ?o))
    )
    :effect
    (
      and
        (not (canTakeObjectCooperant))
        (hasObjectCooperant ?p ?o)
        (not (bagObjectCooperant ?p ?o))
        (canPutObjectCooperant)
    )
  )

  (:action DROP-OBJECT-MAIN
    :parameters (?o - object ?p - main ?z - zone)
    :precondition
    (
      and
        (not (canTakeObjectMain))
        ;(not (canPutObjectMain))
        (atmain ?p ?z)
        (hasObjectMain ?p ?o)
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
        (canTakeObjectMain)
        (not (hasObjectMain ?p ?o))
        ;(not (bagObjectMain ?p ?o))
        (atObject ?o ?z)
        ;(canPutObjectMain)
    )
  )

  (:action DROP-OBJECT-COOPERANT
    :parameters (?o - object ?p - cooperant ?z - zone)
    :precondition
    (
      and
        (not (canTakeObjectCooperant))
        ;(not (canPutObjectCooperant))
        (atcooperant ?p ?z)
        (hasObjectCooperant ?p ?o)
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
        (canTakeObjectCooperant)
        (not (hasObjectCooperant ?p ?o))
        ;(not (bagObjectCooperant ?p ?o))
        (atObject ?o ?z)
        ;(canPutObjectCooperant)
    )
  )

  (:action GIVE-OBJECT-MAIN
       :parameters (?z - zone ?o - object ?p - main ?n - npc)
	     :precondition
	     (
	        and
	            (hasObjectMain ?p ?o)
	            (atmain ?p ?z)
	            (atNPC ?n ?z)
              (< (npcObjects ?n) (npcObjectsTotal ?n))
              (not (hasObjectMain ?p bikini))   ; Si tiene este objeto se lo daría por tanto necesitamos que no se lo de pues nos hace falta
              (not (hasObjectMain ?p zapatilla)); Si tiene este objeto se lo daría por tanto necesitamos que no se lo de pues nos hace falta
	     )
	     :effect
	     (
	        and
              (canTakeObjectMain)
              (increase (npcObjects ?n) 1)
              (not (hasObjectMain ?p ?o))
              (increase (totalScore) (objectScore ?o ?n))
	     )
  )

  (:action GIVE-OBJECT-COOPERANT
       :parameters (?z - zone ?o - object ?p - cooperant ?m - main)
	     :precondition
	     (
	        and
	            (hasObjectCooperant ?p ?o)
	            (atcooperant ?p ?z)
	            (atmain ?m ?z)
              (canTakeObjectMain)
              ;(not (hasObjectCooperant ?p bikini))
              ;(not (hasObjectCooperant ?p zapatilla))
	     )
	     :effect
	     (
	        and
              (canTakeObjectCooperant)
              (not (hasObjectCooperant ?p ?o))
              (hasObjectMain ?m ?o)
              (not (canTakeObjectMain))
	     )
  )
)
