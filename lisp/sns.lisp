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

;;; TODO: use Lisp arrays for message buffers instead of foreign-heap

(define-condition msg-error (error)
  ((message
    :initarg :message)
   (type
    :initarg :type)))


(defun msg-error (type fmt &rest args)
  (error 'msg-error
        :message (apply #'format nil fmt args)
        :type type))

(defconstant +header-size+ (foreign-type-size '(:struct msg-header)))

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

(defmacro with-msg-pointer ((pointer-var &optional (size-var (gensym)))
                            msg &body body)
  (with-gensyms (msg-1)
    `(let ((,msg-1 ,msg))
       (let ((,pointer-var (msg-buffer-pointer ,msg-1))
             (,size-var (msg-buffer-size ,msg-1)))
         (when (< ,size-var  +header-size+)
           (msg-error :overflow "Message buffer (~A) smaller than header size (~A)"
                      ,size-var +header-size+))
         ,@body))))

(defun msg-header (msg slot)
  (with-msg-pointer (p) msg
    (foreign-slot-value p
                        '(:struct msg-header)
                        slot)))

(defun check-var-msg (actual-size obj-size count index)
  (let ((expected-size (+ +header-size+
                          (* obj-size count))))
    (when (< actual-size expected-size)
      (msg-error :overflow "Message buffer (~A) smaller than expected size (~A)"
                 actual-size expected-size)))
  (when (>= index count)
    (msg-error :overflow "Index (~A) out of bounds (~A)"
               index count)))

;; TODO: size checks

(defgeneric msg-size-n (msg n))
(defgeneric msg-size (msg))
(defgeneric msg-alloc (type n))

(defgeneric put-msg (channel msg))
(defgeneric get-msg (channel msg &key wait last))
(defgeneric check-msg (channel))

(defgeneric msg-aref (msg i))
(defgeneric msg-aref (msg i))
(defgeneric msg-aref-ptr (msg i))
(defgeneric msg-decode (type pointer))

(defmethod ach::put-object (channel (object msg-buffer))
  (with-msg-pointer (ptr size) object
    (ach:put-pointer channel ptr size)))

(defmethod put-msg (channel (msg msg-buffer))
  (ach::put-object channel msg))

(defun decode-double (pointer count)
  (let ((v (make-array count :element-type 'double-float)))
    (dotimes (i count)
      (setf (aref v i)
            (mem-aref pointer :double i)))
    v))

(defmacro def-msg-base (type)
  (let ((make-it (intern (concatenate 'string "MAKE-" (string type)))))
    `(progn
       (defstruct (,type (:include msg-buffer)))
       (defmethod get-msg (channel (msg (eql ',type)) &key wait last)
         (multiple-value-bind (pointer r frame-size)
             (ach::get-foreign-alloc channel :wait wait :last last)
           (let ((msg (msg-buffer-initialize (,make-it) pointer frame-size)))
             (check-msg-header msg)
             (values msg r)))))))

(defmacro def-msg-var (type slot)
  (let* ((slot-type (foreign-slot-type `(:struct ,type) slot))
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
        (defmethod msg-aref-ptr ((msg ,type) i)
          (with-msg-pointer (pointer size) msg
            (check-var-msg size (foreign-type-size ',slot-type)
                           (foreign-slot-value pointer '(:struct msg-header) pointer) 'n)
            (let ((base (foreign-slot-pointer pointer '(:struct ,type) ',slot)))
              (mem-aptr base ',slot-type i))))
        (defmethod msg-aref ((msg ,type) i)
          (msg-decode ',type (msg-aref-ptr msg i)))
       )))

(defmethod msg-decode ((type (eql 'quaternion)) pointer)
  (aa::make-quaternion :data (decode-double pointer 4)))

(defmethod msg-decode ((type (eql 'vec3)) pointer)
  (aa::make-vec3 :data (decode-double pointer 3)))

(defcstruct tf
  (r :double :count 4)
  (x :double :count 3))
(defcstruct msg-tf
  (header (:struct msg-header))
  (tf (:struct tf) :count 1))
(def-msg-var msg-tf tf)

(defmethod msg-decode ((type (eql 'msg-tf)) pointer)
  (aa::make-quaternion-translation :quaternion
                                   (msg-decode 'quaternion
                                               (foreign-slot-value pointer '(:struct wt-tf) 'r))
                                   :translation
                                   (msg-decode 'vec3
                                               (foreign-slot-value pointer '(:struct wt-tf) 'x))))

(defcstruct msg-joystick
  (header (:struct msg-header))
  (buttons :uint64)
  (axis :double :count 1))
(def-msg-var msg-joystick axis)

(defcstruct wt-tf
  (r :double :count 4)
  (x :double :count 3)
  (weight :double))
(defcstruct msg-wt-tf
  (header (:struct msg-header))
  (wt-tf (:struct wt-tf) :count 1))
(def-msg-var msg-wt-tf wt-tf)
(defmethod msg-decode ((type (eql 'msg-wt-tf)) pointer)
  (aa::make-quaternion-translation :quaternion
                                   (msg-decode 'quaternion
                                               (foreign-slot-value pointer '(:struct wt-tf) 'r))
                                   :translation
                                   (msg-decode 'vec3
                                               (foreign-slot-value pointer '(:struct wt-tf) 'x))))

(defun get-foreign-string (pointer max)
  (with-output-to-string (s)
    (loop for i below max
       for c = (mem-aref pointer :char i)
       until (zerop c)
       do (write-char (code-char c) s))))

(defun msg-print-header (msg &optional (stream t))
  (with-msg-pointer (ptr) msg
    (labels ((slot (x) (foreign-slot-value ptr '(:struct msg-header) x)))
      (format stream "~&sec: ~A" (slot 'sec))
      (format stream "~&nsec: ~A" (slot 'nsec))
      (format stream "~&dur-nsec: ~A" (slot 'dur-nsec))
      (format stream "~&from-pid: ~A" (slot 'from-pid))
      (format stream "~&from-host: ~A" (get-foreign-string (slot 'from-host) +hostname-len+))
      (format stream "~&ident: ~A" (get-foreign-string (slot 'ident) +ident-len+))
      (format stream "~&n: ~A" (slot 'n))
      )))

(defun rec-edit (input-file output-directory &key filter if-exists zero-time)
  (let* ((data-0 (aa::read-vectors input-file))
         (data (if filter (mapcar filter data-0) data-0))
         (t-0 (cond
                ((eq zero-time t)
                 (elt (elt data 0) 0))
                (t 0)))
         (n (length (car data))))
    (dotimes (i (1- n))
      (aa::write-float-lists (loop for elt in data
                                collect (list (- (aref elt 0) t-0)
                                              (aref elt (1+ i))))
                             (format nil "~A/~D.dat" output-directory i)
                             :if-exists if-exists))))
