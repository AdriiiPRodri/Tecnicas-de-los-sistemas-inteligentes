(define (domain zeno-travel)


(:requirements
  :typing
  :fluents
  :derived-predicates
  :negative-preconditions
  :universal-preconditions
  :disjuntive-preconditions
  :conditional-effects
  :htn-expansion

  ; Requisitos adicionales para el manejo del tiempo
  :durative-actions
  :metatags
 )

(:types aircraft person city - object)
(:constants slow fast - object)
(:predicates (at ?x - (either person aircraft) ?c - city)
             (in ?p - person ?a - aircraft)
             (different ?x ?y) (igual ?x ?y)
             (hay-fuel-slow ?a ?c1 ?c2)
             (hay-fuel-fast ?a ?c1 ?c2)
             (disponible-fuel-slow ?a ?c1 ?c2)
             (disponible-fuel-fast ?a ?c1 ?c2)
             )
(:functions (fuel ?a - aircraft)
            (distance ?c1 - city ?c2 - city)
            (slow-speed ?a - aircraft)
            (fast-speed ?a - aircraft)
            (slow-burn ?a - aircraft)
            (fast-burn ?a - aircraft)
            (capacity ?a - aircraft)
            (refuel-rate ?a - aircraft)
            (total-fuel-used)
            (boarding-time)
            (debarking-time)
            (fuel-limit) ; EJERCICIO 3 este es el límite que se establece en el ejercicio 3
            )

;; el consecuente "vac�o" se representa como "()" y significa "siempre verdad"
(:derived
  (igual ?x ?x) ())

(:derived
  (different ?x ?y) (not (igual ?x ?y)))



;; este literal derivado se utiliza para deducir, a partir de la información en el estado actual,
;; si hay fuel suficiente para que el avión ?a vuele de la ciudad ?c1 a la ?c2
;; el antecedente de este literal derivado comprueba si el fuel actual de ?a es mayor que 1.
;; En este caso es una forma de describir que no hay restricciones de fuel. Pueden introducirse una
;; restricción más copleja  si en lugar de 1 se representa una expresión más elaborada (esto es objeto de
;; los siguientes ejercicios).
(:derived
  (hay-fuel-slow ?a - aircraft ?c1 - city ?c2 - city)
  (> (fuel ?a) (* (distance ?c1 ?c2) (slow-burn ?a)))
)

(:derived
  (hay-fuel-fast ?a - aircraft ?c1 - city ?c2 - city)
  (> (fuel ?a) (* (distance ?c1 ?c2) (fast-burn ?a)))
)

(:derived
  (disponible-fuel-slow ?a - aircraft ?c1 - city ?c2 - city)
  (< (+ (total-fuel-used) (* (distance ?c1 ?c2) (slow-burn ?a))) (fuel-limit))
)

(:derived
  (disponible-fuel-fast ?a - aircraft ?c1 - city ?c2 - city)
  (< (+ (total-fuel-used) (* (distance ?c1 ?c2) (fast-burn ?a))) (fuel-limit))
)

(:task transport-person
	:parameters (?p - person ?c - city)

  (:method Case1 ; si la persona est� en la ciudad no se hace nada
	 :precondition (at ?p ?c)
	 :tasks ()
   )


   (:method Case2 ;si no est� en la ciudad destino, pero avion y persona est�n en la misma ciudad
	  :precondition (and (at ?p - person ?c1 - city)
			                 (at ?a - aircraft ?c1 - city))

	  :tasks (
	  	      (board ?p ?a ?c1)
		        (mover-avion ?a ?c1 ?c)
		        (debark ?p ?a ?c )))

    ; EJERCICIO 1
    (:method Case3 ;si el avión y la persona están en ciudades diferentes y por supuesto la persona no está en la ciudad de destino ya
    :precondition (and (at ?p - person ?c1 - city)
                       (at ?a - aircraft ?c2 - city))

    :tasks (
          (mover-avion ?a ?c2 ?c1) ; Movemos
          (board ?p ?a ?c1) ; Embarcamos
          (mover-avion ?a ?c1 ?c) ; Vamos al destino
          (debark ?p ?a ?c)
          )
    )
	)

(:task mover-avion
 :parameters (?a - aircraft ?c1 - city ?c2 -city)

  ; EJERCICIO 3, he cambiado el orden de los métodos para conseguir que el planificador priorice la velocidad rápida como indica el enunciado del problema
  (:method fuel-suficiente-fast
           :precondition (and (hay-fuel-fast ?a ?c1 ?c2) (disponible-fuel-fast ?a ?c1 ?c2))
           :tasks (
                   (zoom ?a ?c1 ?c2)
                  )
   )
    (:method fuel-insuficiente-fast
           :precondition (not (hay-fuel-fast ?a ?c1 ?c2))
           :tasks(
                 (refuel ?a ?c1)
                 (mover-avion ?a ?c1 ?c2)
                 )
    )

    ; EJERCICIO 2
    (:method fuel-insuficiente-slow
           :precondition (not (hay-fuel-slow ?a ?c1 ?c2))
           :tasks(
                 (refuel ?a ?c1)
                 (mover-avion ?a ?c1 ?c2)
                 )
    )

    (:method fuel-suficiente-slow ;; este método se escogerá para usar la acción fly siempre que el avión tenga fuel para
                             ;; volar desde ?c1 a ?c2
   			  ;; si no hay fuel suficiente el método no se aplicará y la descomposición de esta tarea
   			  ;; se intentará hacer con otro método. Cuando se agotan todos los métodos posibles, la
   			  ;; descomponsición de la tarea mover-avión "fallará".
   			  ;; En consecuencia HTNP hará backtracking y escogerá otra posible vía para descomponer
   			  ;; la tarea mover-avion (por ejemplo, escogiendo otra instanciación para la variable ?a)
     :precondition (and (hay-fuel-slow ?a ?c1 ?c2) (disponible-fuel-slow ?a ?c1 ?c2))
     :tasks (
             (fly ?a ?c1 ?c2)
            )
     )
 )

(:import "Primitivas-Zenotravel.pddl")
)