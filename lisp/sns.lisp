;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2014, Georgia Tech Research Corporation
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ntd@gatech.edu>
;;;; Georgia Tech Humanoid Robotics Lab
;;;; Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
;;;;
;;;;
;;;; This file is provided under the following "BSD-style" License:
;;;;
;;;;
;;;;   Redistribution and use in source and binary forms, with or
;;;;   without modification, are permitted provided that the following
;;;;   conditions are met:
;;;;
;;;;   * Redistributions of source code must retain the above copyright
;;;;     notice, this list of conditions and the following disclaimer.
;;;;
;;;;   * Redistributions in binary form must reproduce the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer in the documentation and/or other materials provided
;;;;     with the distribution.
;;;;
;;;;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
;;;;   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
;;;;   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
;;;;   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;;;;   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
;;;;   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;;;;   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;;;;   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
;;;;   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
;;;;   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
;;;;   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
;;;;   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;;;   POSSIBILITY OF SUCH DAMAGE.


(in-package :sns)


(defstruct msg-buffer
  pointer
  size)

(defun msg-buffer-initialize (b pointer size)
  ;; allocate
  (setf (msg-buffer-pointer b) pointer
        (msg-buffer-size b) size)
  ;; register finalizer
  (sb-ext:finalize b
                   (lambda ()
                     (cffi:foreign-free pointer)))
  b)

(defgeneric msg-size-n (msg n))
(defgeneric msg-size (msg))
(defgeneric msg-alloc (type n))

(defgeneric msg-put (channel msg))
(defgeneric msg-get (channel msg &key wait last))

(defmethod ach::put-object (channel (object msg-buffer))
  (ach:put-pointer channel (msg-buffer-pointer object) (msg-buffer-size object)))

(defmethod msg-put (channel (msg msg-buffer))
  (ach::put-object channel msg))

(defmacro def-msg-base (type)
  (let ((make-it (intern (concatenate 'string "MAKE-" (string type)))))
    `(progn
       (defstruct (,type (:include msg-buffer)))
       (defmethod msg-get (channel (msg (eql ',type)) &key wait last)
         ;; TODO: check message size
         (multiple-value-bind (pointer r frame-size)
             (ach::get-foreign-alloc channel :wait wait :last last)
           (values (msg-buffer-initialize (,make-it) pointer frame-size)
                   r))))))

(defmacro def-msg-var (type slot)
  (let* ((slot-type-raw (foreign-slot-type `(:struct ,type) slot))
         (slot-type (case slot-type-raw
                     ((:double) slot-type-raw)
                     (otherwise (list :struct slot-type-raw))))
        (make-it (intern (concatenate 'string "MAKE-" (string type)))))
    `(progn
       (def-msg-base ,type)
       (defmethod msg-size-n ((msg (eql ',type)) n)
         (+ (foreign-type-size '(:struct ,type))
            (* (- n 1)
               (foreign-type-size ',slot-type))))
       (defmethod msg-alloc ((type (eql ',type)) n)
         (let* ((size (msg-size-n ',type n))
                (pointer (foreign-alloc :uint8 :count size)))
           (msg-buffer-initialize (,make-it)
                                  pointer size)))
       )))


(defcstruct tf
  (r :double :count 4)
  (x :double :count 3))
(defcstruct msg-tf
  (header (:struct msg-header))
  (tf (:struct tf) :count 1))
(def-msg-var msg-tf tf)

(defcstruct msg-joystick
  (header (:struct msg-header))
  (buttons :uint64)
  (axis :double :count 1))
(def-msg-var msg-joystick axis)

(defun get-foreign-string (pointer max)
  (with-output-to-string (s)
    (loop for i below max
       for c = (mem-aref pointer :char i)
       until (zerop c)
       do (write-char (code-char c) s))))

(defun msg-print-header (msg &optional (stream t))
  (assert (>= (msg-buffer-size msg)
              (foreign-type-size '(:struct msg-header))))
  (labels ((slot (x) (foreign-slot-value (msg-buffer-pointer msg) '(:struct msg-header) x)))
    (format stream "~&sec: ~A" (slot 'sec))
    (format stream "~&nsec: ~A" (slot 'nsec))
    (format stream "~&dur-nsec: ~A" (slot 'dur-nsec))
    (format stream "~&from-pid: ~A" (slot 'from-pid))
    (format stream "~&from-host: ~A" (get-foreign-string (slot 'from-host) +hostname-len+))
    (format stream "~&ident: ~A" (get-foreign-string (slot 'ident) +ident-len+))
    (format stream "~&n: ~A" (slot 'n))
    ))
