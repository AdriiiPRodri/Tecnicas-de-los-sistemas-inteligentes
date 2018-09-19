(define (domain monkey-domain)	       ; Comment: adding location caused fail
  (:requirements :strips :equality :typing)
  (:types  monkey banana box - locatable
          location)

  (:predicates
	       (on-floor ?x - monkey )
	       (at ?m - locatable ?x - location)
	       (onbox ?x - monkey ?y - location)
	       (hasbananas ?x - monkey)
  )
  (:action GRAB-BANANAS
	     :parameters (?m - monkey ?y - location ?b - banana)
	     :precondition (and (onbox ?m ?y) (at ?b ?y))
	     :effect (hasbananas ?m))
  (:action GOTO
         :parameters (?m - monkey ?y - location)
         :precondition (and (on-floor ?m))
         :effect (and (at ?m ?y)))
  (:action CLIM-BOX
         :parameters (?m - monkey ?l - location ?b -box)
         :precondition (and (at ?m ?l) (at ?b ?l))
         :effect (and (onbox ?m ?l)))
  (:action PUSH-BOX
         :parameters (?m - monkey ?x1 - location ?b -box ?x2 - location)
         :precondition (and (at ?m ?x1) (at ?b ?x1))
         :effect (and (at ?m ?x2) (at ?b ?x2)))
)
