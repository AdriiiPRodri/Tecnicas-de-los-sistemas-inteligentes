

(define (domain pelis)


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

 ;; Se define un dominio con varias personas, de distinta edad (mayores y menores de 18 años).
 ;; Las personas están inicialmente en una cola para ir al cine
 ;; y hay que decidir qué personas ven una pelicula de terror (mayores de 18) y cuales una pelicula disney (menores de 18)
 
(:types person - object)
(:constants disney terror - object)
(:predicates (edad ?x - person ?e - number) ;; una persona ?x tiene la edad ?e
             (en-cola ?x - person)			;; una persona ?x está en la cola para ir al cine
             )

(:task infiere ;;tarea de mayor nivel de abstracción
 :parameters() 

 (:method recurre1 ;; método para decidir si una persona va a la pelicula de terror.
  :precondition (and (edad ?x ?e) (en-cola ?x) (>= ?e 18))
  :tasks ( (ver-peli-terror ?x) ;; añade accion primitiva al plan
           (infiere)))			;; recurre para decidir la siguiente persona en la cola
 
 (:method recurre2 ;; método para decidir si una persona va a una pelicula disney
  :precondition (and (edad ?x ?e) (en-cola ?x) (< ?e 18))
  :tasks ( (ver-peli-disney ?x) ;;añade la accion primitiva al plan
            (infiere)))			;;recurre para decidir sobre la siguiente persona en la cola
  (:method caso-base ;; método caso base de la recursión
   :precondition():tasks()))

  (:durative-action ver-peli-disney ;;una person deja de estar en la cola para ver una peli disney.
 :parameters (?p - person )
 :duration (= ?duration 2)
 :condition (and  (en-cola ?p) )
                 
 :effect (and  (not (en-cola ?p))))

(:durative-action ver-peli-terror ;;una person deja de estar en la cola para ver una peli de terror.
 :parameters (?p - person )
 :duration (= ?duration 2)
 :condition (and  (en-cola ?p) )
                 
 :effect (and  (not (en-cola ?p))))
 )
