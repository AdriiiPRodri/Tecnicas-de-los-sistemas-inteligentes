﻿(define (problem Ejercicio1)
    (:domain Ejercicio1-domain)
  (:objects z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 z11 z12 z13 z14 z15 z16 z17 z18 z19 z20 z21 z22 z23 z24 z25 - zone
            player1 - player
            princesa principe bruja profesor leonardoDiCaprio - npc
            oscars manzanas algoritmos rosas oro zapatilla bikini - object
            north east south west - orientation
            bosque agua precipicio arena piedra - surface
			)
  (:init

; Indices de nuestras zonas
  ; 1  2  3  4  5
  ; 6  7  8  9  10
  ; 11 12 13 14 15
  ; 16 17 18 19 20
  ; 21 22 23 24 25

; Tipo de suelo de nuestra zona
  ; W  S  S  F  R       W = water  S = sand  F = forest  R = rock  P = precipice
  ; F  F  S  S  P
  ; R  S  S  F  P
  ; W  W  S  S  S
  ; R  W  F  F  W

; Distribución de los objetos
  ; A  N AP  N  N       N = nothing  A = algorithms  AP = apples  G = gold  R = roses  S = shoes  B = bikini  O = oscars
  ; N  G  S  N  N
  ; R  N  N  N  N
  ; N  N  N  N  N
  ; N  N  B  O  N

; Distribución de NPCs
  ; N  P  N  N  N       P = professor  Q = princess  W = witch  K = prince  L = leonardoDiCaprio
  ; N  N  N  Q  N
  ; N  N  W  N  N
  ; N  N  K  N  N
  ; L  N  N  N  N


    (orientedZone z1 east z2)
    (orientedZone z1 south z6)
    (orientedZone z2 east z3)
    (orientedZone z2 south z7)
    (orientedZone z2 west z1)
    (orientedZone z3 east z4)
    (orientedZone z3 south z8)
    (orientedZone z3 west z2)
    (orientedZone z4 east z5)
    (orientedZone z4 south z9)
    (orientedZone z4 west z3)
    (orientedZone z5 south z10)
    (orientedZone z5 west z4)
    (orientedZone z6 north z1)
    (orientedZone z6 east z7)
    (orientedZone z6 south z11)
    (orientedZone z7 north z2)
    (orientedZone z7 east z8)
    (orientedZone z7 south z12)
    (orientedZone z7 west z6)
    (orientedZone z8 north z3)
    (orientedZone z8 east z9)
    (orientedZone z8 south z13)
    (orientedZone z8 west z7)
    (orientedZone z9 north z4)
    (orientedZone z9 east z10)
    (orientedZone z9 south z14)
    (orientedZone z9 west z8)
    (orientedZone z10 north z5)
    (orientedZone z10 south z15)
    (orientedZone z10 west z9)
    (orientedZone z11 north z6)
    (orientedZone z11 east z12)
    (orientedZone z11 south z16)
    (orientedZone z12 north z7)
    (orientedZone z12 east z13)
    (orientedZone z12 south z17)
    (orientedZone z12 west z11)
    (orientedZone z13 north z8)
    (orientedZone z13 east z14)
    (orientedZone z13 south z18)
    (orientedZone z13 west z12)
    (orientedZone z14 north z9)
    (orientedZone z14 east z15)
    (orientedZone z14 south z19)
    (orientedZone z14 west z13)
    (orientedZone z15 north z10)
    (orientedZone z15 south z20)
    (orientedZone z15 west z14)
    (orientedZone z16 north z11)
    (orientedZone z16 east z17)
    (orientedZone z16 south z21)
    (orientedZone z17 north z12)
    (orientedZone z17 east z18)
    (orientedZone z17 south z22)
    (orientedZone z17 west z16)
    (orientedZone z18 north z13)
    (orientedZone z18 east z19)
    (orientedZone z18 south z23)
    (orientedZone z18 west z17)
    (orientedZone z19 north z14)
    (orientedZone z19 east z20)
    (orientedZone z19 south z24)
    (orientedZone z19 west z18)
    (orientedZone z20 north z15)
    (orientedZone z20 south z25)
    (orientedZone z20 west z19)
    (orientedZone z21 north z16)
    (orientedZone z21 east z22)
    (orientedZone z22 north z17)
    (orientedZone z22 east z23)
    (orientedZone z22 west z21)
    (orientedZone z23 north z18)
    (orientedZone z23 east z24)
    (orientedZone z23 west z22)
    (orientedZone z24 north z19)
    (orientedZone z24 east z25)
    (orientedZone z24 west z23)
    (orientedZone z25 north z20)
    (orientedZone z25 west z24)

    (typesurface z1 agua)
    (typesurface z2 arena)
    (typesurface z3 arena)
    (typesurface z4 bosque)
    (typesurface z5 piedra)
    (typesurface z6 bosque)
    (typesurface z7 bosque)
    (typesurface z8 arena)
    (typesurface z9 arena)
    (typesurface z10 precipicio)
    (typesurface z11 piedra)
    (typesurface z12 arena)
    (typesurface z13 arena)
    (typesurface z14 bosque)
    (typesurface z15 precipicio)
    (typesurface z16 agua)
    (typesurface z17 agua)
    (typesurface z18 arena)
    (typesurface z19 arena)
    (typesurface z20 arena)
    (typesurface z21 piedra)
    (typesurface z22 agua)
    (typesurface z23 bosque)
    (typesurface z24 bosque)
    (typesurface z25 agua)

    (= (distance z1 z2) 1)
    (= (distance z1 z6) 1)
    (= (distance z2 z3) 1)
    (= (distance z2 z7) 1)
    (= (distance z2 z1) 1)
    (= (distance z3 z4) 1)
    (= (distance z3 z8) 1)
    (= (distance z3 z2) 1)
    (= (distance z4 z5) 1)
    (= (distance z4 z9) 1)
    (= (distance z4 z3) 1)
    (= (distance z5 z10) 1)
    (= (distance z5 z4) 1)
    (= (distance z6 z1) 1)
    (= (distance z6 z7) 1)
    (= (distance z6 z11) 1)
    (= (distance z7 z2) 1)
    (= (distance z7 z8) 1)
    (= (distance z7 z12) 1)
    (= (distance z7 z6) 1)
    (= (distance z8 z3) 1)
    (= (distance z8 z9) 1)
    (= (distance z8 z13) 1)
    (= (distance z8 z7) 1)
    (= (distance z9 z4) 1)
    (= (distance z9 z10) 1)
    (= (distance z9 z14) 1)
    (= (distance z9 z8) 1)
    (= (distance z10 z5) 1)
    (= (distance z10 z15) 1)
    (= (distance z10 z9) 1)
    (= (distance z11 z6) 1)
    (= (distance z11 z12) 1)
    (= (distance z11 z16) 1)
    (= (distance z12 z7) 1)
    (= (distance z12 z13) 1)
    (= (distance z12 z17) 1)
    (= (distance z12 z11) 1)
    (= (distance z13 z8) 1)
    (= (distance z13 z14) 1)
    (= (distance z13 z18) 1)
    (= (distance z13 z12) 1)
    (= (distance z14 z9) 1)
    (= (distance z14 z15) 1)
    (= (distance z14 z19) 1)
    (= (distance z14 z13) 1)
    (= (distance z15 z10) 1)
    (= (distance z15 z20) 1)
    (= (distance z15 z14) 1)
    (= (distance z16 z11) 1)
    (= (distance z16 z17) 1)
    (= (distance z16 z21) 1)
    (= (distance z17 z12) 1)
    (= (distance z17 z18) 1)
    (= (distance z17 z22) 1)
    (= (distance z17 z16) 1)
    (= (distance z18 z13) 1)
    (= (distance z18 z19) 1)
    (= (distance z18 z23) 1)
    (= (distance z18 z17) 1)
    (= (distance z19 z14) 1)
    (= (distance z19 z20) 1)
    (= (distance z19 z24) 1)
    (= (distance z19 z18) 1)
    (= (distance z20 z15) 1)
    (= (distance z20 z25) 1)
    (= (distance z20 z19) 1)
    (= (distance z21 z16) 1)
    (= (distance z21 z22) 1)
    (= (distance z22 z17) 1)
    (= (distance z22 z23) 1)
    (= (distance z22 z21) 1)
    (= (distance z23 z18) 1)
    (= (distance z23 z24) 1)
    (= (distance z23 z22) 1)
    (= (distance z24 z19) 1)
    (= (distance z24 z25) 1)
    (= (distance z24 z23) 1)
    (= (distance z25 z20) 1)
    (= (distance z25 z24) 1)

    ;Puntos de Leonardo
    (= (objectScore oscars leonardoDiCaprio) 10)
    (= (objectScore rosas leonardoDiCaprio) 1)
    (= (objectScore manzanas leonardoDiCaprio) 3)
    (= (objectScore algoritmos leonardoDiCaprio) 4)
    (= (objectScore oro leonardoDiCaprio) 5)
    ;Puntos de la princesa
    (= (objectScore oscars princesa) 5)
    (= (objectScore rosas princesa) 10)
    (= (objectScore manzanas princesa) 1)
    (= (objectScore algoritmos princesa) 3)
    (= (objectScore oro princesa) 4)
    ;Puntos de la bruja
    (= (objectScore oscars bruja) 4)
    (= (objectScore rosas bruja) 5)
    (= (objectScore manzanas bruja) 10)
    (= (objectScore algoritmos bruja) 1)
    (= (objectScore oro bruja) 3)
    ;Puntos del profesor
    (= (objectScore oscars profesor) 3)
    (= (objectScore rosas profesor) 4)
    (= (objectScore manzanas profesor) 5)
    (= (objectScore algoritmos profesor) 10)
    (= (objectScore oro profesor) 1)
    ;Puntos del principe
    (= (objectScore oscars principe) 1)
    (= (objectScore rosas principe) 3)
    (= (objectScore manzanas principe) 4)
    (= (objectScore algoritmos principe) 5)
    (= (objectScore oro principe) 10)

    (= (total-distance) 0)
    (canTakeObject)
    (canPutObject)
    (= (totalScore) 0)

  (atPlayer player1 z3)
  (actualOrientation north player1)
  (atNPC princesa z9)
  (atNPC principe z18)
  (atNPC bruja z13)
  (atNPC profesor z2)
  (atNPC leonardoDiCaprio z21)
  (atObject manzanas z3)
  (atObject oscars z24)
  (atObject algoritmos z1)
  (atObject rosas z11)
  (atObject oro z7)
  (atObject bikini z23)
  (atObject zapatilla z8)
	 )
  (:goal (AND
              (>= (totalScore) 40) ;
         )
  )

  ;(:metric maximize (totalScore))
)