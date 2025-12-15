import { useState } from "react";
import AppHeader from "@/components/AppHeader";
import DocumentsHeader from "@/components/documents-management/DocumentsHeader";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogTrigger, DialogFooter } from "@/components/ui/dialog";
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table";
import { Eye, Trash2, Plus, FileText, Upload } from "lucide-react";

// Dados mockados
const INITIAL_DOCS = [
  { id: 1, title: "lorem ipsum", date: "00/00/0000" },
  { id: 2, title: "lorem ipsum", date: "00/00/0000" },
  { id: 3, title: "lorem ipsum", date: "00/00/0000" },
  { id: 4, title: "lorem ipsum", date: "00/00/0000" },
  { id: 5, title: "lorem ipsum", date: "00/00/0000" },
  { id: 6, title: "lorem ipsum", date: "00/00/0000" },
  { id: 7, title: "lorem ipsum", date: "00/00/0000" },
  { id: 8, title: "lorem ipsum", date: "00/00/0000" },
  { id: 9, title: "lorem ipsum", date: "00/00/0000" },
  { id: 10, title: "lorem ipsum", date: "00/00/0000" },
];

const DocumentsManagement = () => {
  const [docs, setDocs] = useState(INITIAL_DOCS);
  const [isDialogOpen, setIsDialogOpen] = useState(false);
  const [newDocName, setNewDocName] = useState("");

  const handleDelete = (id: number) => {
    setDocs(docs.filter(doc => doc.id !== id));
  };

  const handleUpload = () => {
    console.log("Upload:", newDocName);
    setIsDialogOpen(false);
    setNewDocName("");
  };

  const handleRoteiroUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (file) {
      console.log("Roteiro selecionado:", file.name);
    }
  };

  return ( 
    <div className="h-screen bg-background flex flex-col overflow-hidden">
      <AppHeader/>
      <DocumentsHeader />
      
      <main className="flex-1 max-w-7xl w-full mx-auto px-6 py-8 overflow-hidden">
        <div className="flex flex-col lg:flex-row gap-8 h-full">
          
          {/* Coluna Esquerda: Lista de Documentos */}
          <div className="flex-1 flex flex-col h-full relative">
            <h2 className="text-xl font-medium mb-4 text-foreground">Documentos</h2>
            
            <div className="relative flex-1 flex flex-col min-h-0">
              
              {/* Botão Flutuante */}
              <div className="absolute -right-5 -top-5 z-50">
                <Dialog open={isDialogOpen} onOpenChange={setIsDialogOpen}>
                  <DialogTrigger asChild>
                    <Button 
                      size="icon" 
                      className="h-12 w-12 rounded-full bg-primary hover:bg-primary/90 shadow-lg"
                    >
                      <Plus className="h-6 w-6 text-white" />
                    </Button>
                  </DialogTrigger>
                  <DialogContent className="sm:max-w-[425px]">
                    <DialogHeader>
                      <DialogTitle>Adicionar Documento</DialogTitle>
                    </DialogHeader>
                    <div className="grid gap-4 py-4">
                      <div className="grid gap-2">
                        <Label htmlFor="name">Nome do documento</Label>
                        <Input
                          id="name"
                          value={newDocName}
                          onChange={(e) => setNewDocName(e.target.value)}
                          placeholder="Digite o nome do arquivo"
                        />
                      </div>
                      <div className="grid gap-2">
                        <Label htmlFor="file">Arquivo</Label>
                        <div className="flex items-center justify-center w-full">
                          <label htmlFor="file-upload" className="flex flex-col items-center justify-center w-full h-32 border-2 border-dashed rounded-lg cursor-pointer hover:bg-gray-50 border-gray-300">
                            <div className="flex flex-col items-center justify-center pt-5 pb-6">
                              <Upload className="w-8 h-8 mb-2 text-gray-400" />
                              <p className="text-sm text-gray-500">Clique para selecionar</p>
                            </div>
                            <input id="file-upload" type="file" className="hidden" />
                          </label>
                        </div>
                      </div>
                    </div>
                    <DialogFooter>
                      <Button onClick={handleUpload}>Salvar documento</Button>
                    </DialogFooter>
                  </DialogContent>
                </Dialog>
              </div>

              {/* Card Branco da Tabela */}
              <div className="bg-white rounded-3xl border border-gray-200 shadow-sm flex-1 overflow-hidden flex flex-col relative z-0">
                <div className="overflow-auto flex-1">
                  <Table>
                    <TableHeader className="sticky top-0 bg-[#8B5CF6] z-0">
                      <TableRow className="border-none hover:bg-[#8B5CF6]">
                        <TableHead className="text-white font-medium pl-8 rounded-tl-lg">Título</TableHead>
                        <TableHead className="text-white font-medium text-center">Data de upload</TableHead>
                        <TableHead className="text-white font-medium text-center rounded-tr-lg pr-8">Ações</TableHead>
                      </TableRow>
                    </TableHeader>
                    <TableBody>
                      {docs.map((doc) => (
                        <TableRow key={doc.id} className="border-b border-gray-100 hover:bg-gray-50">
                          <TableCell className="font-medium pl-8 py-3 text-gray-600">{doc.title}</TableCell>
                          <TableCell className="text-center py-3 text-gray-600">{doc.date}</TableCell>
                          <TableCell className="text-center py-3 pr-8">
                            <div className="flex items-center justify-center gap-4">
                              <button className="text-gray-900 hover:text-primary transition-colors">
                                <Eye className="w-5 h-5" />
                              </button>
                              <button 
                                onClick={() => handleDelete(doc.id)}
                                className="text-gray-900 hover:text-red-500 transition-colors"
                              >
                                <Trash2 className="w-4 h-4" />
                              </button>
                            </div>
                          </TableCell>
                        </TableRow>
                      ))}
                    </TableBody>
                  </Table>
                </div>
              </div>
            </div>
          </div>

          {/* Coluna Direita: Roteiro */}
          <div className="flex-1 flex flex-col h-full">
            <h2 className="text-xl font-medium mb-4 text-foreground">Roteiro</h2>
            <div className="flex-1 bg-[#F5F3FF] border-2 border-[#E9D5FF] rounded-3xl flex items-center justify-center">
              
              {/* script seleção de arquivo */}
              <input 
                type="file" 
                id="roteiro-upload-trigger" 
                className="hidden" 
                onChange={handleRoteiroUpload}
              />
              
              {/* botão, disparando o input de seleção de arquivo */}
              <label 
                htmlFor="roteiro-upload-trigger"
                className="bg-[#A855F7] p-4 rounded-xl shadow-sm cursor-pointer hover:bg-[#9333EA] transition-all duration-200 hover:scale-105 active:scale-95"
                title="Clique para carregar um roteiro"
              >
                <FileText className="w-8 h-8 text-white" />
              </label>

            </div>
          </div>

        </div>
      </main>
    </div>
  );
};

export default DocumentsManagement;