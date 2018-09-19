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
             (destino ?x - person ?y - city) ; Ejercicio 4
             )
(:functions (fuel ?a - aircraft)
            (distance ?c1 - city ?c2 - city)
            (slow-speed ?a - aircraft)
            (fast-speed ?a - aircraft)
            (slow-burn ?a - aircraft)
            (fast-burn ?a - aircraft)
            (capacity ?a - aircraft)
            (refuel-rate ?a - aircraft)
            (total-fuel-used ?a - aircraft)
            (boarding-time)
            (debarking-time)
            (fuel-limit ?a - aircraft) ; EJERCICIO 3 este es el límite que se establece en el ejercicio 3, modificado pues podemos tener varios aviones
            (disponible-duracion-fast ?a - aircraft ?c1 - city ?c2 - city) ; Ejercicio 4, para la duración limitada de los viajes
            (disponible-duracion-slow ?a - aircraft ?c1 - city ?c2 - city) ; Ejercicio 4, para la duración limitada de los viajes
            (numero-pasajeros ?a - aircraft)
            (max-pasajeros ?a - aircraft)
            (duracion ?a - aircraft) ; Duración actual del avión
            (max-duracion ?a - aircraft) ; Máxima duración del avión
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
  (< (+ (total-fuel-used ?a) (* (distance ?c1 ?c2) (slow-burn ?a))) (fuel-limit ?a))
)

(:derived
  (disponible-fuel-fast ?a - aircraft ?c1 - city ?c2 - city)
  (< (+ (total-fuel-used ?a) (* (distance ?c1 ?c2) (fast-burn ?a))) (fuel-limit ?a))
)

(:derived
  (disponible-duracion-fast ?a - aircraft ?c1 - city ?c2 - city)
  (<= (+ (duracion ?a) (/ (distance ?c1 ?c2) (fast-speed ?a))) (max-duracion ?a))
)

(:derived
  (disponible-duracion-slow ?a - aircraft ?c1 - city ?c2 - city)
  (<= (+ (duracion ?a) (/ (distance ?c1 ?c2) (slow-speed ?a))) (max-duracion ?a))
)

; Ejercicio 4
(:task board-all
 :parameters ()
			 (:method Case1
       :precondition (and (at ?p - person ?c - city)
                     (at ?a - aircraft ?c - city)
                     (not (destino ?p - person ?c - city))
                     (< (numero-pasajeros ?a - aircraft) (max-pasajeros ?a - aircraft)))
							:tasks (
                      (board ?p ?a ?c1)
                      (board-all))
                      )
			 (:method Case2 :precondition ()
						    :tasks ())
)

(:task transport-person
:parameters (?p - person ?c - city)
    (:method Case1
                  :precondition (and (at ?p ?c) (destino ?p ?c))
                  :tasks ()
    )

    (:method Case2
                  :precondition (and (at ?p - person ?c1 - city) (at ?a - aircraft ?c1 - city) (not (destino ?p - person ?c1 - city)))
                  :tasks ((board-all) (mover-avion ?a ?c1 ?c) (transport-person ?p ?c))
    )

    (:method Case3
                  :precondition (and (destino ?p - person ?c1 - city))
                  :tasks ((debark ?p ?a ?c1) (transport-person ?p ?c))
    )

    (:method Case4
                  :precondition (and (at ?p - person ?c1 - city) (at ?a - aircraft ?c2 - city) (different ?c1 ?c2))
                  :tasks ((mover-avion ?a ?c2 ?c1) (transport-person ?p ?c))
    )

    (:method Case5
                  :precondition (and (in ?p - person ?a - aircraft) (at ?a - aircraft ?c1 - city) (destino ?p - person ?c2 - city) (different ?c1 ?c2))
                  :tasks ((mover-avion ?a ?c1 ?c2) (transport-person ?p ?c2))
    )
)

(:task mover-avion
 :parameters (?a - aircraft ?c1 - city ?c2 -city)

  ; EJERCICIO 3, he cambiado el orden de los métodos para conseguir que el planificador priorice la velocidad rápida como indica el enunciado del problema
  (:method fuel-suficiente-fast
           :precondition (and (hay-fuel-fast ?a ?c1 ?c2) (disponible-fuel-fast ?a ?c1 ?c2) (disponible-duracion-fast ?a ?c1 ?c2))
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

    (:method fuel-suficiente-slow ;; este método se escogerá para usar la acción fly siempre que el avión tenga fuel para
                             ;; volar desde ?c1 a ?c2
   			  ;; si no hay fuel suficiente el método no se aplicará y la descomposición de esta tarea
   			  ;; se intentará hacer con otro método. Cuando se agotan todos los métodos posibles, la
   			  ;; descomponsición de la tarea mover-avión "fallará".
   			  ;; En consecuencia HTNP hará backtracking y escogerá otra posible vía para descomponer
   			  ;; la tarea mover-avion (por ejemplo, escogiendo otra instanciación para la variable ?a)
     :precondition (and (hay-fuel-slow ?a ?c1 ?c2) (disponible-fuel-slow ?a ?c1 ?c2) (disponible-duracion-slow ?a ?c1 ?c2))
     :tasks (
             (fly ?a ?c1 ?c2)
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

 )

(:import "Primitivas-Zenotravel-4.pddl")
)
