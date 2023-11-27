#ifndef LINKEDLIST_H
#define LINKEDLIST_H

#include <iostream>

template <typename T>
class ListNode
{
  public:
    T data;
    ListNode* next;

    ListNode(T data)
    {
        this->data = data;
        this->next = nullptr;
    }
};

template <typename T>
class LinkedList
{
  private:
    ListNode<T>* head;

  public:

    LinkedList();

    void insertAtBeginning(T data);
    void insertAtEnd(T data);
    void deleteAtBeginning();
    void deleteAtEnd();
    void printList();
};
#endif // LINKEDLIST_H
